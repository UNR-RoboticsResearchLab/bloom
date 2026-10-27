"""Private GPU service using the pinned AutoRSR scoring implementation."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import uuid

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / '.runtime' / 'autorsr'
MODEL = os.environ.get('ARSR_MODEL', 'large-v3')
_model = None
_compute_type = None
_analysis_lock = threading.Lock()


def get_model():
    global _model, _compute_type
    if _model is None:
        import ctranslate2
        import whisperx
        supported = ctranslate2.get_supported_compute_types('cuda')
        requested = os.environ.get('ARSR_COMPUTE_TYPE', 'auto')
        _compute_type = next((t for t in ('float16', 'int8_float32', 'float32') if t in supported), None) if requested == 'auto' else requested
        if _compute_type not in supported:
            raise RuntimeError('The requested CUDA compute type is unsupported.')
        print(f'Loading WhisperX {MODEL} on CUDA, compute_type={_compute_type}, batch_size=1', flush=True)
        # WhisperX 3.1.1 predates these faster-whisper 1.0.3 option fields.
        # Supply their normal defaults explicitly rather than changing decoding.
        _model = whisperx.load_model(MODEL, 'cuda', compute_type=_compute_type, language='en',
            asr_options={'max_new_tokens': None, 'clip_timestamps': '0',
                         'hallucination_silence_threshold': None, 'hotwords': None})
    return _model


def transcribe(path):
    import whisperx
    result = get_model().transcribe(whisperx.load_audio(str(path)), batch_size=1)
    return ' '.join(s['text'].strip() for s in result.get('segments', []) if 'text' in s)


def bounded_score(response, target):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('score_worker.py')),
        str(RUNTIME / 'source/auto_rsr.py')], input=json.dumps({'response': response, 'target': target}),
        text=True, capture_output=True, check=True, timeout=4)
    return json.loads(result.stdout)


def validate_markers(markers):
    if not isinstance(markers, list) or not markers or len(markers) > 16:
        raise ValueError('Provide 1 to 16 response windows.')
    seen = set()
    for marker in markers:
        item = marker['sentence_id']
        start, end = marker['start_ms'], marker['end_ms']
        if type(item) is not int or not 1 <= item <= 16 or item in seen:
            raise ValueError('Response item IDs must be unique integers from 1 to 16.')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (start, end)) or not 0 <= start < end:
            raise ValueError('Response windows must have finite increasing times.')
        seen.add(item)
    return {m['sentence_id']: m for m in markers}


def create_app(scorer=None, transcriber=transcribe):
    from flask import Flask, request, jsonify
    isolated_scoring = scorer is None
    if scorer is None:
        sys.path.insert(0, str(RUNTIME / 'source'))
        import auto_rsr as scorer
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 200_000_000
    manifest = json.loads((ROOT / 'robot/scripts/rsr_sentences.json').read_text())
    recordings = Path(os.environ.get('ARSR_RECORDINGS_DIR', RUNTIME / 'recordings'))
    recordings.mkdir(parents=True, exist_ok=True, mode=0o700)

    @app.get('/health')
    def health():
        return jsonify(ok=True, model=MODEL, modelLoaded=_model is not None, device='cuda', computeType=_compute_type)

    @app.post('/api/analyze')
    def analyze():
        try:
            age, percentile = int(request.form['age']), int(request.form['percentile'])
            if not 60 <= age <= 119 or percentile not in (5, 10, 15):
                raise ValueError('Use age 60–119 months and percentile 5, 10, or 15.')
            markers = validate_markers(json.loads(request.form['markers']))
            upload = request.files['session_audio']
        except (KeyError, ValueError, TypeError):
            return jsonify(error='Invalid audio, age, percentile, or response windows.'), 400
        # A single P4 serves one assessment at a time; never load duplicate models.
        if not _analysis_lock.acquire(blocking=False):
            return jsonify(error='AutoRSR is processing another recording. Retry shortly.'), 503
        try:
            folder = recordings / uuid.uuid4().hex
            folder.mkdir(mode=0o700)
            session = folder / 'session.webm'
            upload.save(session)
            sentences, paths = [], {}
            for item in manifest:
                item_id, gt = item['id'], item['text']
                marker = markers.get(item_id)
                if marker is None:
                    continue
                clip = folder / f'sentence_{item_id}.wav'
                # Match the original AutoRSR service's two-second pre-roll.
                subprocess.run(['ffmpeg', '-nostdin', '-y', '-i', str(session),
                    '-ss', str(max(0, marker['start_ms'] - 2000) / 1000),
                    '-to', str(marker['end_ms'] / 1000), str(clip)],
                    check=True, capture_output=True, timeout=120)
                response = scorer.standarize(transcriber(clip))
                try:
                    target = scorer.standarize(gt).split()
                    if isolated_scoring:
                        scored = bounded_score(response.split(), target)
                    else:
                        edits = scorer.score_rsr_errors(response.split(), target)
                        errors = sum(len(v) for v in edits.values())
                        scored = dict(errors=errors, score=scorer.score_rsr([errors]), editScript=edits)
                    sentences.append(dict(id=str(item_id), groundTruth=gt, response=response, **scored))
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                    sentences.append(dict(id=str(item_id), groundTruth=gt, response=response,
                        errors=None, score=None, editScript=None,
                        analysisError='Edit search exceeded its resource limit. Review this item manually.'))
                paths[str(item_id)] = str(clip)
            total = sum(s['score'] for s in sentences if s['score'] is not None)
            incomplete = [s['id'] for s in sentences if s['score'] is None]
            decision = ('Incomplete — manual review required for items ' + ', '.join(incomplete)) if incomplete else scorer.evaluate_rsr_result(total, age, percentile)
            return jsonify(totalScore=total, result=decision,
                sentences=sentences, sessionFolder=str(folder), recordingPaths=paths)
        except Exception:
            app.logger.exception('AutoRSR analysis failed')
            # Infrastructure/transcription failures must never become a child's zero score.
            return jsonify(error='AutoRSR could not complete analysis. Recording and manual overrides are retained.'), 503
        finally:
            _analysis_lock.release()
    return app


if __name__ == '__main__':
    from waitress import serve
    os.umask(0o077)
    os.chdir(RUNTIME / 'source')  # AutoRSR's normalizer loads english.json relative to cwd.
    app = create_app()
    get_model()  # Download/warm before accepting any assessment requests.
    serve(app, host=os.environ.get('ARSR_HOST', '127.0.0.1'), port=int(os.environ.get('ARSR_PORT', '5050')), threads=2)
