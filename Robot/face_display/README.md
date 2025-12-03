# Face Display Module

Browser-based face with viseme support for lip-sync

## Attribution

Original code by Denielle Oliva: https://github.com/denielleoliva/blsm_unr  
Modified for Bloom speech therapy robot project

## What It Does

Shows an animated face in a web browser with:
- Eyes that blink automatically and track the mouse
- Emotion-based expressions (happy, sad, excited, etc)
- Viseme-based mouth shapes for lip-sync with Azure TTS

## Files

- `blossom_face.html` - The face display
- `test_face.py` - Opens face and shows keyboard controls
- `test_viseme_timing.py` - Example of how timing works 
- `README.md` - This file

## Quick Test
```bash
# Open face in browser
python test_face.py

# Or just open the HTML directly
open blossom_face.html
```

## Keyboard Controls

When face is open in browser:

**Emotions:**
- 1-8: Switch emotions (happy, sad, excited, etc)
- Space: Blink
- C: Toggle control panel

**Viseme Testing:**
- V: Switch to viseme mode
- E: Switch back to emotion mode  
- T: Test all mouth shapes
- S: Print current status to console
- Arrow keys: Cycle through visemes manually

## How It Works

The face has two modes:

**Emotion Mode (default):**
- Mouth shows smile/frown based on emotion
- Used when robot isn't speaking

**Viseme Mode (during speech):**
- Mouth shows phoneme shapes that match the audio
- Switches shapes based on viseme timeline from TTS

## Viseme Data Format

Azure TTS returns viseme data like this:
```python
visemes = [
    {"viseme_id": 0, "audio_offset": 0},
    {"viseme_id": 21, "audio_offset": 1000000},
    {"viseme_id": 4, "audio_offset": 1500000}
]
```

The face supports 22 Azure viseme IDs mapped to 9 mouth shapes:
- 0: Closed (silence, m/p/b sounds)
- 1-3: Open (ah, eh, uh sounds)
- 4-5: Medium open (cat, bed sounds)  
- 6-9: Smile shapes (ee, ay sounds)
- 10-14: Rounded (oo, oh sounds)
- 15-17: Narrow vertical (f, v, th)
- 18-19: Narrow horizontal (t, d, s, z)
- 20-21: Puckered (sh, ch)

Use arrow keys in viseme mode to see each shape.

## Notes

- Face runs in browser, no server needed for basic testing
- Mouth shapes are simplified - only 9 shapes for all 22 visemes
- No teeth or tongue rendered
- Background color changes with emotion