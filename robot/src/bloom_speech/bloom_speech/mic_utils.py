"""Shared microphone device discovery for the bloom_speech STT nodes.

stt_node.py (Azure) and stt_node_vosk.py (offline Vosk) both used to trust
whatever the OS considered the "default" capture device. That's unreliable
on the robot: a USB mic array like the ReSpeaker Lite doesn't always become
the system default, and which device *is* default can silently change after
a reboot or replug - so STT ends up listening to the wrong device (or none)
with no error, just silence.

Both nodes now look up the ReSpeaker explicitly by name instead.

Set BLOOM_MIC_DEVICE to override the lookup:
  - a substring of the device name (e.g. "ReSpeaker" or "Lite"), matched
    case-insensitively against what ALSA/PortAudio report
  - a plain integer, treated as a PortAudio device index (find_sounddevice_input only)
Unset, both default to matching "respeaker".
"""
import os
import subprocess

DEFAULT_NAME_HINT = 'respeaker'


def _name_hint():
    override = os.environ.get('BLOOM_MIC_DEVICE', '').strip()
    if override and not override.isdigit():
        return override
    return DEFAULT_NAME_HINT


def find_alsa_capture_device(logger=None):
    """Find an ALSA PCM name for Azure SpeechSDK's AudioConfig(device_name=...).

    Parses `arecord -L`, which lists PCM tokens flush-left with an indented
    description line underneath, e.g.:
        plughw:CARD=Lite,DEV=0
            ReSpeaker Lite, USB Audio, playback/capture

    Returns a token like "plughw:CARD=Lite,DEV=0", or None if nothing
    matched (caller should fall back to use_default_microphone=True).
    """
    hint = _name_hint()
    try:
        out = subprocess.run(
            ['arecord', '-L'], capture_output=True, text=True, timeout=5
        ).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
        if logger:
            logger.warn(f'[mic] could not run `arecord -L`: {e}')
        return None

    lines = out.splitlines()
    candidates = []
    for i, line in enumerate(lines):
        if not line.strip() or line.startswith(' '):
            continue
        token = line.strip()
        if not token.startswith('plughw:'):
            continue
        desc = lines[i + 1].strip() if i + 1 < len(lines) else ''
        if hint.lower() in token.lower() or hint.lower() in desc.lower():
            candidates.append((token, desc))

    if not candidates:
        if logger:
            logger.warn(
                f"[mic] no ALSA capture device matching '{hint}' found via "
                "`arecord -L` - is the ReSpeaker Lite plugged in?"
            )
        return None

    device, desc = candidates[0]
    if logger:
        logger.info(f"[mic] using ALSA device '{device}' ({desc})")
    return device


def find_sounddevice_input(logger=None):
    """Find a PortAudio input device index for sounddevice.RawInputStream.

    Returns (index, info_dict), or (None, None) if nothing matched (caller
    should fall back to PortAudio's default device).
    """
    import sounddevice as sd

    devices = sd.query_devices()
    override = os.environ.get('BLOOM_MIC_DEVICE', '').strip()

    if override.isdigit():
        idx = int(override)
        if 0 <= idx < len(devices) and devices[idx]['max_input_channels'] > 0:
            if logger:
                logger.info(
                    f"[mic] using device #{idx} '{devices[idx]['name']}' "
                    "(forced by BLOOM_MIC_DEVICE)"
                )
            return idx, devices[idx]
        if logger:
            logger.warn(
                f'[mic] BLOOM_MIC_DEVICE={override} is not a valid input '
                'device index'
            )
        return None, None

    hint = _name_hint()
    for idx, info in enumerate(devices):
        if info['max_input_channels'] > 0 and hint.lower() in info['name'].lower():
            if logger:
                logger.info(f"[mic] using device #{idx} '{info['name']}'")
            return idx, info

    if logger:
        logger.warn(
            f"[mic] no input device matching '{hint}' found - falling back "
            "to PortAudio's default device"
        )
    return None, None
