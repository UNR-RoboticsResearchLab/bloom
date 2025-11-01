# TTS Module:

## Prerequisites:
- Python 3.x

This module provides Text-to-Speech (TTS) functionality with viseme (mouth-shape timeline) output. For now the system is using Azure TTS, but this folder contains an interface in case latency proves to be too much and another system needs to be tested.

## Features:
- Uses Azure Speech SDK for TTS
- Generates an audio file (default: BloomResponse.wav) per synthesis
- Captures viseme timing events: For each viseme (mouth shape of phoneme), returns a timestamp (in milliseconds) and a viseme ID for driving facial animations
    - [Phoneme to viseme mapping](https://learn.microsoft.com/en-us/azure/ai-services/speech-service/how-to-speech-synthesis-viseme?tabs=visemeid&pivots=programming-language-csharp#map-phonemes-to-visemes)
- TTSEngineInterface: allows TTS engine to be swapped out in case latency is a problem
- For now credentials are hardcoded for testing, will switch to a config later

## Getting started:

1. Install dependencies:
    - Navigate to tts_module:
    ```bash
    cd ~/bloom/Robot/tts_module
    ```

    - Create (or activate) your Python virual environment:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

    - Install the required Python packages from requirements.txt:
    ```bash
    pip install -r requirements.txt
    ```

2. Edit test_engine.py and set subscription_key, region, and optionally default_voice
    - subscription_key and region are found under your Azure Speech Services Keys and Endpoints

3. Run the test script:
    ```bash
    python test_engine.py --text "Hello robot, this is a test."
    ```

4. This script should output:
    - Path to audio file (BlossomResponse.wav)
    - A list of viseme events (timestamp in ms -> viseme ID)

5. Play the audio to verify

## Usage in Code:

```python
from engine_azure import AzureTTSEngine

engine = AzureTTSEngine(
    subscription_key="YOUR_KEY",
    region="YOUR_REGION",
    default_voice="en-US-AvaNeural"
)

result = engine.synthesize(text="Hello world", voice_id=None, include_viseme=True)

print("Audio file:", result.audio_filepath)
print("Number of viseme events:", len(result.viseme_events))

print("Metrics:")
print("  Total latency (ms):", result.metrics.total_latency_ms)
print("  First-byte latency (ms):", result.metrics.first_byte_latency_ms)
print("  Network latency (ms):", result.metrics.network_latency_ms)
print("  Success:", result.metrics.success)
if not result.metrics.success:
    print("  Error reason:", result.metrics.error_reason)

"""
#Audio File Behavior
# result.audio_filepath -> path to WAV file
# result.viseme_events -> list of (timestamp_ms, viseme_id)

#Performance Metrics
# result.total_latency_ms -> time in milliseconds from request start to completion of speech output
# result.first_byte_latency_ms -> time in milliseconds from request start to arrival of the first audio byte
# result.network_latency_ms -> time in milliseconds of the network/API round trip
# result.num_viseme_events -> count of viseme events captured (for mouth animation mapping)
# result.success -> boolean flag indicating whether synthesis succeeded
# result.error_reason -> string describing what failed (if success=false)
"""
```

Your animation subsystem can then loop through result.viseme_events, schedule mouth animations by timestamp_ms offset, and play result.audio_filepath

## Performance Metrics:
The module returns performance data with each synthesis request, allowing for metrics tracking and performance measuring.

- total_latency_ms - time in milliseconds from when synthesis was requested to when the audio file is ready 
- first_byte_latency_ms —> time in milliseconds from request start to when the first audio byte is returned
- network_latency_ms -> time in milliseconds representing the network/API round-trip
- num_viseme_events -> count of viseme events captured (useful for verifying mouth animation alignment)
- success -> boolean flag indicating if synthesis succeeded
- error_reason -> string describing the failure reason (if success = false)

## Config Considerations:

- Filename: Currently the audio file is saved under BlossomResponse.wav . If needed this can be changed to a timestamped or versioned filename for logging and history keeping.
- Overwriting: By default, each synthesis of text overwrites the same filename in order to prevent accumulating too many files, this can be changed with the above point
 


