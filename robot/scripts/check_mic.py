#!/usr/bin/env python3
"""Standalone microphone diagnostic for the Bloom robot's ReSpeaker Lite.

Run this directly on the robot - no ROS2, no bloom_speech build needed -
to figure out which layer is broken:
  1. Does Linux/ALSA see the ReSpeaker Lite as a capture device at all?
     (a driver/USB/permissions problem shows up here)
  2. Can PortAudio (sounddevice - what stt_node_vosk and this script use)
     open and read from it?
  3. Is it actually picking up sound? (records a few seconds, reports level)

Usage:
    python3 check_mic.py [name_hint]

name_hint defaults to "respeaker" (case-insensitive substring match against
device names), or reads BLOOM_MIC_DEVICE the same way the ROS2 stt nodes do.
"""
import os
import subprocess
import sys


def check_alsa(hint):
    print('== ALSA capture devices (arecord -l) ==')
    try:
        subprocess.run(['arecord', '-l'], check=False)
    except FileNotFoundError:
        print('  arecord not found - install alsa-utils (apt install alsa-utils)')
        return
    print()
    print('== ALSA PCM device names (arecord -L) ==')
    out = subprocess.run(['arecord', '-L'], capture_output=True, text=True).stdout
    print(out)
    if hint.lower() not in out.lower():
        print(f"  !! nothing matching '{hint}' found here.")
        print('     Check the USB connection and run `dmesg | tail -30`')
        print('     right after plugging it in to look for enumeration errors.')


def check_portaudio(hint):
    print()
    print('== PortAudio / sounddevice devices ==')
    try:
        import sounddevice as sd
    except ImportError:
        print('  sounddevice not installed (pip install sounddevice)')
        return None
    except OSError as e:
        print(f'  sounddevice failed to load: {e}')
        print('  (likely missing the PortAudio library - apt install libportaudio2)')
        return None
    devices = sd.query_devices()
    match_idx = None
    for i, d in enumerate(devices):
        marker = ''
        if d['max_input_channels'] > 0 and hint.lower() in d['name'].lower():
            marker = '  <-- match'
            match_idx = i
        print(f"  [{i}] {d['name']!r}  in={d['max_input_channels']} "
              f"out={d['max_output_channels']} default_sr={d['default_samplerate']}{marker}")
    if match_idx is None:
        print(f"  !! no PortAudio input device matches '{hint}'.")
        print('     If step 1 found it in ALSA but this list is missing it,')
        print("     it's a PortAudio/ALSA-plugin issue, not a hardware one.")
    return match_idx


def record_and_measure(index, seconds=3, samplerate=16000):
    import numpy as np
    import sounddevice as sd
    label = index if index is not None else '(PortAudio default)'
    print()
    print(f'== Recording {seconds}s from device {label} - make some noise ==')
    audio = sd.rec(int(seconds * samplerate), samplerate=samplerate, channels=1,
                    dtype='int16', device=index)
    sd.wait()
    rms = float(np.sqrt(np.mean(audio.astype(np.float32) ** 2)))
    peak = int(np.max(np.abs(audio)))
    print(f'  RMS={rms:.1f}  peak={peak}  (int16 range is 0-32767)')
    if peak < 200:
        print('  !! near-silence. Check mic gain/mute, the cable, and that')
        print('     nothing else (another process) has the device open exclusively.')
    else:
        print('  looks like audio is coming through.')

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mic_check_recording.wav')
    try:
        import soundfile as sf
        sf.write(out_path, audio, samplerate)
        print(f'  wrote {out_path} - play it back (e.g. `aplay {out_path}`) to confirm it sounds right')
    except ImportError:
        print('  (install soundfile to also save a .wav you can play back)')


def main():
    hint = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('BLOOM_MIC_DEVICE', 'respeaker')
    print(f"Looking for a device matching '{hint}'\n")
    check_alsa(hint)
    idx = check_portaudio(hint)
    try:
        record_and_measure(idx)
    except Exception as e:
        print(f'\n  recording failed: {e}')


if __name__ == '__main__':
    main()
