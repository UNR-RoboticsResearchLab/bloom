# STT Module

## Prerequisites:
- Python 3.x
- Azure Speech Services subscription key (default engine)
- OpenAI API key (optional, for Whisper engine)

This module provides Speech-to-Text (STT) functionality for the Bloom robot. The system uses Azure Speech-to-Text by default, but includes an interface to swap engines if needed (e.g., switching to OpenAI Whisper or a local model for lower latency or offline capability).

## Features:
- Azure Speech-to-Text engine (default, uses Azure student credits)
- OpenAI Whisper API engine (alternative, requires OpenAI credits)
- Supports both file-based transcription and microphone recording
- Returns performance metrics (latency, audio duration, success/failure)
- STTEngineInterface: allows STT engine to be swapped out if needed
- Microphone device selection and testing utility

## Getting Started:

### 1. Install dependencies

Navigate to stt_module:
```bash
cd ~/bloom/Robot/stt_module
```

Create (or activate) your Python virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the required Python packages from requirements.txt:
```bash
pip install -r requirements.txt
```

### 2. Configure API credentials

Copy the example config file:
```bash
cp config_example.py config.py
```

Edit config.py and add your credentials:
```python
# Azure credentials (required for default engine)
AZURE_SUBSCRIPTION_KEY = "your-azure-key-here"
AZURE_REGION = "westus2"

# OpenAI credentials (optional, only needed if using Whisper engine)
OPENAI_API_KEY = "your-openai-key-here"
```

Get your Azure credentials from: Azure Speech Services Keys and Endpoints

Get your OpenAI API key from: https://platform.openai.com/api-keys

### 3. Set up your microphone

Before running the STT module, set up your microphone:
```bash
python setup_mic.py
```

This will:
1. List all available microphones
2. Let you test which one works
3. Save your preference to config.py

### 4. Run the test script
```bash
python test_engine.py
```

This script should output:
- Transcribed text from your speech
- Performance metrics (latency, audio duration)

## Switching Between Engines:

The module uses **Azure Speech-to-Text by default**. To switch to the Whisper engine:

1. Open `test_engine.py`
2. Comment out the Azure engine and uncomment the Whisper engine:
```python
# Option 1: Azure STT (uses Azure student credits)
# ENGINE = AzureSTTEngine(
#     subscription_key=AZURE_SUBSCRIPTION_KEY,
#     region=AZURE_REGION,
#     input_device=MICROPHONE_DEVICE
# )

# Option 2: OpenAI Whisper (requires OpenAI credits)
ENGINE = WhisperSTTEngine(
    api_key=OPENAI_API_KEY,
    model="whisper-1"
)
```

3. Make sure you've added your OpenAI API key to config.py

## Troubleshooting Microphone Issues:

### "No audio detected" or "Max volume: 0.0000"

This usually means microphone permissions aren't granted. Fix:

**macOS Permissions:**
- Go to: System Settings -> Privacy & Security -> Microphone
- Make sure Terminal (or your IDE) has permission checked
- If not listed, run `setup_mic.py` to trigger the permission dialog

**Wrong microphone selected:**
- Run `python setup_mic.py` to see all devices
- Test each one to find which works
- Save your preference

**Microphone volume too low:**
- System Settings -> Sound -> Input
- Select your microphone
- Increase Input volume slider to 75-100%
- Speak and watch the input level bars

### "Unable to determine number of input channels"

Your system can't detect the microphone. Try:
- Check microphone is plugged in (if external)
- Run `python setup_mic.py` to list available devices
- Grant microphone permissions (see above)

### Still not working?

Run the setup utility again:
```bash
python setup_mic.py
```

Test each available device until you find one that works

## Usage in Code:
```python
from engine_azure import AzureSTTEngine

engine = AzureSTTEngine(
    subscription_key="YOUR_AZURE_KEY",
    region="westus2"
)

# Option 1: Transcribe from microphone
result = engine.transcribe_from_mic(duration_seconds=5.0, language="en-US")

# Option 2: Transcribe from file
result = engine.transcribe_file(audio_filepath="audio.wav", language="en-US")

print("Transcribed text:", result.text)
print("Metrics:")
print("  Total latency (ms):", result.metrics.total_latency_ms)
print("  Audio duration (ms):", result.metrics.audio_duration_ms)
print("  Success:", result.metrics.success)
if not result.metrics.success:
    print("  Error reason:", result.metrics.error_reason)
```

## Performance Metrics:

The module returns performance data with each transcription request, allowing for metrics tracking and performance monitoring

- **total_latency_ms** - time in milliseconds from when transcription was requested to when the text is ready
- **audio_duration_ms** - duration of the audio that was transcribed
- **success** - boolean flag indicating if transcription succeeded
- **error_reason** - string describing the failure reason (if success = false)


## Technical Details:

- **Temporary file:** Currently saves recordings as `temp_recording.wav` and deletes after transcription
- **Sample rate:** 16kHz mono (good enough for speech, reduces file size)
- **Audio format:** 16-bit PCM WAV (required by Azure, also used for Whisper for consistency)
- **Language:** Can choose language code (e.g., "en-US" for American English) for better accuracy