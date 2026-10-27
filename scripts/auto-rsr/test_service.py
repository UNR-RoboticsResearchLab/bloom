import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
from unittest.mock import Mock

spec = importlib.util.spec_from_file_location('study_autorsr', Path(__file__).with_name('service.py'))
service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(service)


class ScorerTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, ARSR_RECORDINGS_DIR=self.folder.name)
        self.env.start()
        self.scorer = types.SimpleNamespace(
            standarize=lambda text: text.lower(),
            score_rsr_errors=lambda response, target: {'Insertions': [], 'Deletions': [], 'Substitutions': [], 'Swaps': []},
            score_rsr=lambda errors: 2,
            evaluate_rsr_result=lambda total, age, percentile: 'N/A')

    def tearDown(self):
        self.env.stop()
        self.folder.cleanup()

    def post(self, client, markers=None):
        return client.post('/api/analyze', data={
            'age': '72', 'percentile': '15',
            'markers': json.dumps(markers if markers is not None else [{'sentence_id': 1, 'start_ms': 2000, 'end_ms': 4000}]),
            'session_audio': (io.BytesIO(b'full recording'), 'session.webm', 'audio/webm;codecs=opus'),
        })

    @patch.object(service.subprocess, 'run')
    def test_browser_upload_returns_only_analyzed_items_and_retains_full_audio(self, ffmpeg):
        client = service.create_app(self.scorer, lambda path: 'Recognized speech').test_client()
        result = self.post(client)
        self.assertEqual(200, result.status_code)
        self.assertEqual(['1'], [s['id'] for s in result.json['sentences']])
        self.assertEqual('recognized speech', result.json['sentences'][0]['response'])
        self.assertEqual(b'full recording', next(Path(self.folder.name).glob('*/session.webm')).read_bytes())
        self.assertEqual('0.0', ffmpeg.call_args.args[0][ffmpeg.call_args.args[0].index('-ss') + 1])  # Original two-second pre-roll.

    @patch.object(service.subprocess, 'run')
    def test_transcription_failure_is_an_error_not_zero_score(self, ffmpeg):
        def broken(path):
            raise RuntimeError('GPU unavailable')
        result = self.post(service.create_app(self.scorer, broken).test_client())
        self.assertEqual(503, result.status_code)
        self.assertNotIn('totalScore', result.json)
        self.assertTrue(list(Path(self.folder.name).glob('*/session.webm')))

    def test_invalid_and_duplicate_windows_are_rejected(self):
        client = service.create_app(self.scorer, lambda path: '').test_client()
        for markers in ([], [{'sentence_id': 1, 'start_ms': 20, 'end_ms': 10}],
                        [{'sentence_id': 1, 'start_ms': 0, 'end_ms': 10}] * 2):
            self.assertEqual(400, self.post(client, markers).status_code)

    def test_concurrent_assessment_is_retriable_without_loading_another_model(self):
        client = service.create_app(self.scorer, lambda path: '').test_client()
        service._analysis_lock.acquire()
        try:
            self.assertEqual(503, self.post(client).status_code)
        finally:
            service._analysis_lock.release()

    def test_p4_uses_supported_precision_and_compatible_decoding_options(self):
        whisper = types.SimpleNamespace(load_model=Mock(return_value=object()))
        ct2 = types.SimpleNamespace(get_supported_compute_types=lambda device: {'float32', 'int8_float32'})
        with patch.dict('sys.modules', whisperx=whisper, ctranslate2=ct2), patch.object(service, '_model', None), patch.dict(os.environ, ARSR_COMPUTE_TYPE='auto'):
            service.get_model()
        options = whisper.load_model.call_args.kwargs
        self.assertEqual('int8_float32', options['compute_type'])
        self.assertEqual({'max_new_tokens', 'clip_timestamps', 'hallucination_silence_threshold', 'hotwords'}, set(options['asr_options']))

    @patch.object(service.subprocess, 'run')
    def test_edit_search_timeout_preserves_transcript_and_marks_item_unscored(self, ffmpeg):
        with patch.dict('sys.modules', auto_rsr=self.scorer), patch.object(service, 'bounded_score', side_effect=service.subprocess.TimeoutExpired('scorer', 4)):
            client = service.create_app(None, lambda path: 'Recognized response').test_client()
            result = self.post(client)
        self.assertEqual(200, result.status_code)
        self.assertIsNone(result.json['sentences'][0]['score'])
        self.assertEqual('recognized response', result.json['sentences'][0]['response'])
        self.assertIn('Incomplete', result.json['result'])
        self.assertIn('analysisError', result.json['sentences'][0])


if __name__ == '__main__':
    unittest.main()
