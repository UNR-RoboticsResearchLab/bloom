"""
Quick test to understand how viseme timing works
Just prints out what happens during playback
"""

import time

# Example of what TTS returns
visemes = [
    {"viseme_id": 0, "audio_offset": 0},
    {"viseme_id": 21, "audio_offset": 1000000},
    {"viseme_id": 4, "audio_offset": 1500000},
    {"viseme_id": 18, "audio_offset": 2500000},
    {"viseme_id": 14, "audio_offset": 3500000},
    {"viseme_id": 0, "audio_offset": 5000000}
]

print("Viseme data example (for 'Hello'):")
print()
for v in visemes:
    time_ms = v["audio_offset"] / 10000
    print(f"{time_ms}ms - viseme {v['viseme_id']}")

print()
print("How this works in the orchestrator:")
print("1. TTS gives us viseme data like above")
print("2. We send it to the HTML face")
print("3. Face and audio start at same time")
print("4. Face checks time and updates mouth automatically")
print("5. Stays in sync because same start time")