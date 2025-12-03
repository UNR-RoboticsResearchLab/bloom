from engine_azure import AzureSTTEngine
from engine_whisper import WhisperSTTEngine
from config import AZURE_SUBSCRIPTION_KEY, AZURE_REGION, OPENAI_API_KEY

# Try to load microphone device from config
try:
    from config import MICROPHONE_DEVICE
except ImportError:
    MICROPHONE_DEVICE = None

# ===== CHOOSE YOUR ENGINE HERE =====

# Show microphone info
import sounddevice as sd
print("\n" + "="*60)
print("Microphone detection")
print("="*60)
default_device = sd.query_devices(kind='input')
print(f"Default input device: {default_device['name']}")
if MICROPHONE_DEVICE is not None:
    print(f"Config override: Device {MICROPHONE_DEVICE}")
    print(f"  {sd.query_devices(MICROPHONE_DEVICE)['name']}")
print("="*60 + "\n")

# Option 1: Azure STT (uses Azure student credits)
ENGINE = AzureSTTEngine(
    subscription_key=AZURE_SUBSCRIPTION_KEY,
    region=AZURE_REGION,
    input_device=MICROPHONE_DEVICE  # Will use config value or auto-detect
)

# Option 2: OpenAI Whisper (requires OpenAI credits)
# ENGINE = WhisperSTTEngine(
#     api_key=OPENAI_API_KEY,
#     model="whisper-1"
# )

# ===================================

"""
Test transcribing an existing audio file
"""
def test_file_transcription():
    
    # Replace with path to your test audio file
    audio_path = "test_audio.wav"
    
    print("Testing file transcription...")
    print(f"Using engine: {ENGINE.__class__.__name__}")
    
    result = ENGINE.transcribe_file(audio_filepath=audio_path, language="en-US")
    
    if result.metrics.success:
        print("\nTranscription successful!")
        print(f"Text: {result.text}")
        print(f"\nMetrics:")
        print(f"  Total latency: {result.metrics.total_latency_ms:.2f} ms")
        print(f"  Audio duration: {result.metrics.audio_duration_ms:.2f} ms")
    else:
        print(f"\nTranscription failed: {result.metrics.error_reason}")

"""
Test recording from microphone and transcribing
"""     
def test_mic_transcription():
    
    print("Testing microphone transcription...")
    print(f"Using engine: {ENGINE.__class__.__name__}")
    print("Speak after you see 'Recording...'")
    
    result = ENGINE.transcribe_from_mic(duration_seconds=5.0, language="en-US")
    
    if result.metrics.success:
        print("\nTranscription successful!")
        print(f"Text: {result.text}")
        print(f"\nMetrics:")
        print(f"  Total latency: {result.metrics.total_latency_ms:.2f} ms")
        print(f"  Audio duration: {result.metrics.audio_duration_ms:.2f} ms")
    else:
        print(f"\nTranscription failed: {result.metrics.error_reason}")

if __name__ == "__main__":
    # Uncomment the test you want to run:
    
    # test_file_transcription()
    test_mic_transcription()