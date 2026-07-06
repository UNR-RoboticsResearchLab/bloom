# Bloom Conversation Robot

Speech therapy robot that has natural conversations with children using Azure AI services

## What It Does

Bloom listens, thinks of responses using GPT, and talks back with animated facial expressions and lip-sync.


## Project Structure
```
/Robot/
|-- conversation_orchestrator.py    # Main program
|-- test_viseme_sync.py            # Test lip-sync timing
|-- requirements.txt               # All dependencies
|--face_display/                  # Browser-based animated face
|-- stt_module/                    # Speech-to-text (Azure)
|-- tts_module/                    # Text-to-speech (Azure)
|-- llm_module/                    # Language model (Azure OpenAI)
```

Each module has its own README with details.

## Setup

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```


Installs everything: pygame, Azure Speech SDK, Azure OpenAI, audio libraries.

For individual modules, each has its own `requirements.txt`.

```bash
git submodule update --init --recursive
```

### 2. Configure API Keys

Set up Azure credentials in each module:
```bash
# STT
cp stt_module/config_example.py stt_module/config.py
# Edit with your keys

# TTS  
cp tts_module/config_example.py tts_module/config.py
# Edit with your keys

# LLM
cp llm_module/config_example.py llm_module/config.py
# Edit with your keys
```

See each module's README for where to get API keys.

### 3. Set Up Microphone (if needed)
```bash
cd stt_module
python setup_mic.py
```

## Running Bloom
```bash
python conversation_orchestrator.py
```

What happens:
1. Face opens in browser
2. HTTP server starts (for face communication)
3. Bloom says "Hi! I'm Bloom..."
4. Speak when you see "Listening..."
5. Bloom responds with animated face
6. Press Ctrl+C to stop


## Systemd Service

sudo cp robot/bloom.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bloom
sudo systemctl start bloom


## How It Works

**Flow:**
```
Speech → STT → LLM → TTS → Face
              ↑
         (maintains history)
```

1. **Listen**: Azure STT transcribes your speech
2. **Think**: GPT generates response based on conversation
3. **Speak**: Azure TTS creates audio + viseme data
4. **Animate**: Face shows emotions and syncs mouth

**Face States:**
- Listening: Calm
- Thinking: Eyes look to side
- Speaking: Happy with lip-sync

**Communication:** Python writes to `face_control.json`, browser polls every 100ms

## Bloom's Personality

System prompt in `conversation_orchestrator.py` defines behavior:
- Patient and encouraging
- Asks follow-up questions
- Short responses (2-3 sentences)
- Natural conversation flow

Edit the prompt to change personality.

## Testing Individual Modules

Test each part before running full system:
```bash
# Test STT
cd stt_module && python test_engine.py

# Test TTS
cd tts_module && python test_engine.py

# Test LLM
cd llm_module && python test_engine.py

# Test face
cd face_display && python test_face.py

# Test lip-sync
python test_viseme_sync.py
```

## Module Overview

**STT Module** - Azure Speech Services for transcription  
**TTS Module** - Azure TTS with viseme data for lip-sync  
**LLM Module** - Azure OpenAI (GPT-4o-mini) for conversation  
**Face Display** - HTML/JS face with 9 mouth shapes

See individual module READMEs for details.

## Troubleshooting

**Face doesn't open:**
- Try manually: http://localhost:8000/blossom_face.html
- Check terminal for HTTP server errors

**No audio:**
- Verify pygame installed
- Check speaker volume
- Verify TTS config

**Microphone issues:**
- Check permissions (System Settings → Privacy)
- Run `stt_module/setup_mic.py`
- Test with `stt_module/test_engine.py`

**LLM errors:**
- Verify API key and deployment name
- Check Azure quota/credits

**Face not responding:**
- Make sure opened via HTTP (not file://)
- Check browser console (F12)
- Verify `face_control.json` exists in face_display/

## Files

- `conversation_orchestrator.py` - Main program
- `test_viseme_sync.py` - Lip-sync testing utility
- `requirements.txt` - All dependencies
- `face_display/` - Face display module
- `stt_module/` - Speech recognition module
- `tts_module/` - Speech synthesis module
- `llm_module/` - Language model module

## Credits

Face display based on work by Denielle Oliva: https://github.com/denielleoliva/blsm_unr

Uses Azure AI services.