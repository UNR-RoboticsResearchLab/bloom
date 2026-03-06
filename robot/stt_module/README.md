# STT Module

Speech-to-Text for Bloom robot using Azure Speech Services or OpenAI Whisper

## Setup

### 1. Install dependencies
```bash
cd stt_module
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Configure API keys
```bash
cp config_example.py config.py
```

Edit `config.py` with your credentials:
- Azure: Get keys from Azure Speech Services
- OpenAI: Get API key from https://platform.openai.com/api-keys

### 3. Set up microphone
```bash
python setup_mic.py
```

This will:
- List available microphones
- Let you test which one works
- Save your preference to config.py

### 4. Test
```bash
python test_engine.py
```

## Engines

**Azure Speech-to-Text (default):** Uses Azure student credits  
**OpenAI Whisper (alternative):** Requires OpenAI API key

Switch engines in `test_engine.py` by commenting/uncommenting the ENGINE initialization.

## Basic Usage
```python
from engine_azure import AzureSTTEngine

engine = AzureSTTEngine(
    subscription_key="your-key",
    region="westus2"
)

# Record from microphone
result = engine.transcribe_from_mic(duration_seconds=5.0, language="en-US")

# Or transcribe from file
result = engine.transcribe_file(audio_filepath="audio.wav", language="en-US")

print("Text:", result.text)
print(f"Latency: {result.metrics.total_latency_ms}ms")
print(f"Success: {result.metrics.success}")
```

## Microphone Issues

If you get "No audio detected" or low volume:

1. **Check permissions:**
   - macOS: System Settings -> Privacy & Security -> Microphone
   - Grant permission to Terminal/IDE

2. **Wrong device selected:**
```bash
   python setup_mic.py  # Test each device
```

3. **Volume too low:**
   - System Settings -> Sound -> Input
   - Increase input volume to 75-100%

4. **Still not working:**
   - Run setup_mic.py and test each available device
   - Check microphone is plugged in (if external)

## Metrics

Results include:
- `text` - Transcribed text
- `metrics.total_latency_ms` - Response time
- `metrics.audio_duration_ms` - Audio length
- `metrics.success` - Success flag
- `metrics.error_reason` - Error details if failed

## Technical Details

- Sample rate: 16kHz mono
- Audio format: 16-bit PCM WAV
- Temporary file: `temp_recording.wav` (auto-deleted)
- Language codes: e.g., "en-US", "es-ES", "fr-FR"

## Files

- `engine_azure.py` - Azure Speech implementation
- `engine_whisper.py` - Whisper implementation
- `engine_interface.py` - Abstract interface
- `test_engine.py` - Test script
- `setup_mic.py` - Microphone setup utility
- `config_example.py` - Template config
- `requirements.txt` - Dependencies