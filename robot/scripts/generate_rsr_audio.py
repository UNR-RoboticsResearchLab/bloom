#!/usr/bin/env python3
"""
One-time offline generation of RSR assessment sentence audio + viseme
timelines, using the same Azure TTS engine the robot uses for lessons.

Run from anywhere:
    python robot/scripts/generate_rsr_audio.py [--output-dir DIR]

Requires robot/tts_module/config.py (copy from config_example.py and fill
in AZURE_SUBSCRIPTION_KEY / AZURE_REGION) — same credentials used elsewhere
in robot/.
"""
import argparse
import json
import os
import shutil
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_OUTPUT_DIR = os.path.join(ROBOT_DIR, "..", "backend", "wwwroot", "rsr-audio")
SENTENCES_PATH = os.path.join(SCRIPT_DIR, "rsr_sentences.json")

sys.path.insert(0, ROBOT_DIR)

from tts_module.config import AZURE_SUBSCRIPTION_KEY, AZURE_REGION  # noqa: E402
from tts_module.engine_azure import AzureTTSEngine  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory to write sentence_NN.wav / sentence_NN.visemes.json / manifest.json into "
             "(default: backend/wwwroot/rsr-audio, served by the backend as static files).",
    )
    args = parser.parse_args()
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    with open(SENTENCES_PATH) as f:
        sentences = json.load(f)

    engine = AzureTTSEngine(subscription_key=AZURE_SUBSCRIPTION_KEY, region=AZURE_REGION)

    manifest = []
    for sentence in sentences:
        sentence_id = sentence["id"]
        text = sentence["text"]
        slug = f"sentence_{sentence_id:02d}"
        print(f"Synthesizing {slug}: {text!r}")

        result = engine.synthesize(text, include_viseme=True)
        if not result.metrics.success or not result.audio_filepath:
            raise RuntimeError(f"TTS failed for sentence {sentence_id}: {result.metrics.error_reason}")

        # engine_azure.py always writes to the same fixed filename, so move it
        # out of the way before synthesizing the next sentence.
        audio_dest = os.path.join(output_dir, f"{slug}.wav")
        shutil.move(result.audio_filepath, audio_dest)

        viseme_dest = os.path.join(output_dir, f"{slug}.visemes.json")
        viseme_events = [
            {"timestamp_ms": e.timestamp_ms, "viseme_id": int(e.viseme_id)}
            for e in result.viseme_events
        ]
        with open(viseme_dest, "w") as f:
            json.dump(viseme_events, f)

        manifest.append({
            "id": sentence_id,
            "text": text,
            "audioUrl": f"/rsr-audio/{slug}.wav",
            "visemeUrl": f"/rsr-audio/{slug}.visemes.json",
        })

    manifest_path = os.path.join(output_dir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nWrote {len(manifest)} sentences to {output_dir}")


if __name__ == "__main__":
    main()
