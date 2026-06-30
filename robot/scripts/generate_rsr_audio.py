#!/usr/bin/env python3
"""
One-time offline generation of RSR assessment sentence audio + viseme
timelines, using the same Azure TTS engine the robot uses for lessons.

Run from anywhere:
    python robot/scripts/generate_rsr_audio.py [--output-dir DIR] [--recordings-dir DIR]

Requires robot/tts_module/config.py (copy from config_example.py and fill
in AZURE_SUBSCRIPTION_KEY / AZURE_REGION) — same credentials used elsewhere
in robot/.

To use your own recorded audio instead of Azure TTS for a sentence, drop a
file named sentence_NN.<ext> (matching the sentence's id in rsr_sentences.json,
e.g. sentence_03.wav) into the recordings directory (default:
robot/scripts/recordings/). Supported formats are whatever soundfile/libsndfile
reads — wav, flac, ogg (not mp3). The recording is used as-is for audioUrl;
Azure is still called for that sentence's text to get a viseme timeline, which
is then time-stretched to match the recording's actual duration, since a
recording has no viseme timing of its own. Sentences without a recording fall
back to full Azure synthesis (audio + visemes), unchanged.
"""
import argparse
import json
import os
import shutil
import sys

import soundfile as sf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_OUTPUT_DIR = os.path.join(ROBOT_DIR, "..", "backend", "wwwroot", "rsr-audio")
DEFAULT_RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
SENTENCES_PATH = os.path.join(SCRIPT_DIR, "rsr_sentences.json")
RECORDING_EXTENSIONS = [".wav", ".flac", ".ogg"]

sys.path.insert(0, ROBOT_DIR)

from tts_module.config import AZURE_SUBSCRIPTION_KEY, AZURE_REGION  # noqa: E402
from tts_module.engine_azure import AzureTTSEngine  # noqa: E402


def find_recording(recordings_dir, sentence_id):
    slug = f"sentence_{sentence_id:02d}"
    for ext in RECORDING_EXTENSIONS:
        path = os.path.join(recordings_dir, f"{slug}{ext}")
        if os.path.exists(path):
            return path
    return None


def duration_ms(audio_path):
    info = sf.info(audio_path)
    return (info.frames / info.samplerate) * 1000.0


def rescale_visemes(viseme_events, original_duration_ms, target_duration_ms):
    if original_duration_ms <= 0:
        return viseme_events
    ratio = target_duration_ms / original_duration_ms
    return [
        {"timestamp_ms": round(e["timestamp_ms"] * ratio), "viseme_id": e["viseme_id"]}
        for e in viseme_events
    ]


def synthesize_visemes(engine, text):
    """Synthesizes text via Azure purely to get viseme timing; returns
    (viseme_events, azure_audio_duration_ms). The Azure audio itself is discarded."""
    result = engine.synthesize(text, include_viseme=True)
    if not result.metrics.success or not result.audio_filepath:
        raise RuntimeError(f"TTS failed: {result.metrics.error_reason}")

    azure_duration = duration_ms(result.audio_filepath)
    os.remove(result.audio_filepath)

    viseme_events = [
        {"timestamp_ms": e.timestamp_ms, "viseme_id": int(e.viseme_id)}
        for e in result.viseme_events
    ]
    return viseme_events, azure_duration


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory to write sentence_NN.<ext> / sentence_NN.visemes.json / manifest.json into "
             "(default: backend/wwwroot/rsr-audio, served by the backend as static files).",
    )
    parser.add_argument(
        "--recordings-dir",
        default=DEFAULT_RECORDINGS_DIR,
        help="Directory to look for sentence_NN.<ext> recordings in before falling back to Azure TTS "
             "(default: robot/scripts/recordings).",
    )
    args = parser.parse_args()
    output_dir = os.path.abspath(args.output_dir)
    recordings_dir = os.path.abspath(args.recordings_dir)
    os.makedirs(output_dir, exist_ok=True)

    with open(SENTENCES_PATH) as f:
        sentences = json.load(f)

    engine = AzureTTSEngine(subscription_key=AZURE_SUBSCRIPTION_KEY, region=AZURE_REGION)

    manifest = []
    for sentence in sentences:
        sentence_id = sentence["id"]
        text = sentence["text"]
        slug = f"sentence_{sentence_id:02d}"

        recording_path = find_recording(recordings_dir, sentence_id)
        if recording_path:
            ext = os.path.splitext(recording_path)[1]
            print(f"{slug}: using recording {os.path.basename(recording_path)}")

            audio_dest = os.path.join(output_dir, f"{slug}{ext}")
            shutil.copyfile(recording_path, audio_dest)

            viseme_events, azure_duration = synthesize_visemes(engine, text)
            recording_duration = duration_ms(audio_dest)
            viseme_events = rescale_visemes(viseme_events, azure_duration, recording_duration)
        else:
            print(f"Synthesizing {slug}: {text!r}")
            result = engine.synthesize(text, include_viseme=True)
            if not result.metrics.success or not result.audio_filepath:
                raise RuntimeError(f"TTS failed for sentence {sentence_id}: {result.metrics.error_reason}")

            # engine_azure.py always writes to the same fixed filename, so move it
            # out of the way before synthesizing the next sentence.
            ext = ".wav"
            audio_dest = os.path.join(output_dir, f"{slug}{ext}")
            shutil.move(result.audio_filepath, audio_dest)

            viseme_events = [
                {"timestamp_ms": e.timestamp_ms, "viseme_id": int(e.viseme_id)}
                for e in result.viseme_events
            ]

        viseme_dest = os.path.join(output_dir, f"{slug}.visemes.json")
        with open(viseme_dest, "w") as f:
            json.dump(viseme_events, f)

        manifest.append({
            "id": sentence_id,
            "text": text,
            "audioUrl": f"/rsr-audio/{slug}{ext}",
            "visemeUrl": f"/rsr-audio/{slug}.visemes.json",
        })

    manifest_path = os.path.join(output_dir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nWrote {len(manifest)} sentences to {output_dir}")


if __name__ == "__main__":
    main()
