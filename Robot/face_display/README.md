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

- `blossom_face.html` - The face (open in browser)
- `test_face.py` - Test script
- `test_viseme_timing.py` - Shows how timing works 
- `README.md` 

## Quick Test

```bash
# Open the face
python test_face.py

# Or open directly
open blossom_face.html
```

## Keyboard Controls

**Emotions:**
- 1-8: Change emotions (happy, sad, etc)
- Space: Blink
- C: Show/hide controls

**Visemes:**
- V: Switch to viseme mode
- E: Switch to emotion mode  
- T: Test all 9 mouth shapes
- S: Show status (current mode and viseme)
- Arrow keys: Cycle through visemes 0-21

## How It Works

The face has two modes:

**Emotion Mode** (default):
- Mouth shows curved smile/frown based on emotion
- Used when robot is not speaking

**Viseme Mode** (during speech):
- Mouth shows phoneme shapes synchronized to audio
- Used when robot is speaking with TTS

## Viseme Support

The face supports 22 Azure visemes mapped to 9 mouth shapes:
- Closed (silence, m/p/b)
- Wide open (father)
- Medium open (cat, bed)
- Narrow smile (eat, day)
- Rounded (boot, go)
- Diphthong (loud, boy)
- Narrow vertical (f, v, th)
- Narrow horizontal (t, d, s, z)
- Puckered (ship, chip)

Press Arrow Right/Left in viseme mode to see all visemes


Example viseme data from Azure TTS:
```python
visemes = [
    {"viseme_id": 0, "audio_offset": 0},
    {"viseme_id": 21, "audio_offset": 1000000},
    {"viseme_id": 4, "audio_offset": 1500000}
]
```


## Notes

- Face runs in browser 
- Viseme shapes are simplified (9 shapes for 22 visemes)
- No teeth/tongue details (consistent simple style)
- Emotion still controls eyes and background during viseme mode