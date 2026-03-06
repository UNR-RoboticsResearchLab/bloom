# TTS Module

Text-to-Speech with viseme data for lip-sync animation

## Setup

### 1. Install dependencies
```bash
cd tts_module
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Configure Azure credentials
```bash
cp config_example.py config.py
```

Edit `config.py` with your Azure credentials:
- Get keys from Azure Speech Services Keys and Endpoints
- Choose a voice from [Azure TTS voices](https://learn.microsoft.com/en-us/azure/ai-services/speech-service/language-support?tabs=tts)

### 3. Test
```bash
python test_engine.py
```

This will:
- Generate audio file (BloomResponse.wav)
- Print viseme events with timestamps

Play the audio to verify: `afplay BloomResponse.wav`

## Features

- **Audio synthesis:** Generates WAV file from text
- **Viseme data:** Returns mouth-shape timeline for animation
- **Performance metrics:** Tracks latency and network timing
- **Swappable engine:** Interface allows switching to local/Pi model if needed

## Basic Usage
```python
from engine_azure import AzureTTSEngine

engine = AzureTTSEngine(
    subscription_key="your-key",
    region="westus2",
    default_voice="en-US-AvaNeural"
)

result = engine.synthesize(
    text="Hello world",
    voice_id=None,
    include_viseme=True
)

print("Audio file:", result.audio_filepath)
print(f"Generated {len(result.viseme_events)} visemes")

# Access viseme timeline
for event in result.viseme_events:
    print(f"{event.timestamp_ms}ms -> viseme {event.viseme_id}")
```

## Viseme Data Format

Each viseme event contains:
- `timestamp_ms` - When this mouth shape should appear (milliseconds into audio)
- `viseme_id` - Azure viseme ID (0-21)

Azure's viseme IDs map to phonemes. See the [phoneme-to-viseme mapping](https://learn.microsoft.com/en-us/azure/ai-services/speech-service/how-to-speech-synthesis-viseme?tabs=visemeid&pivots=programming-language-csharp#map-phonemes-to-visemes).

## Metrics

Results include:
- `audio_filepath` - Path to generated audio file
- `viseme_events` - List of viseme timing events
- `metrics.total_latency_ms` - Time from request to completion
- `metrics.first_byte_latency_ms` - Time to first audio byte
- `metrics.network_latency_ms` - Network round-trip time
- `metrics.num_viseme_events` - Count of viseme events
- `metrics.success` - Success flag
- `metrics.error_reason` - Error details if failed

## Technical Details

- **Output file:** BloomResponse.wav (overwrites each synthesis)
- **Audio format:** WAV, 16-bit PCM
- **Viseme timing:** Timestamps in 100-nanosecond units, converted to milliseconds
- **Default voice:** en-US-AvaNeural (Neural voice for natural speech)

## Files

- `engine_azure.py` - Azure TTS implementation
- `engine_interface.py` - Abstract interface
- `test_engine.py` - Test script
- `config_example.py` - Template config
- `requirements.txt` - Dependencies