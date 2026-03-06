from .engine_azure import AzureTTSEngine
from .config import AZURE_SUBSCRIPTION_KEY, AZURE_REGION, AZURE_DEFAULT_VOICE

def main():
    engine = AzureTTSEngine(
        subscription_key=AZURE_SUBSCRIPTION_KEY,
        region=AZURE_REGION,
        default_voice=AZURE_DEFAULT_VOICE
    )
    text = "Hello robot, this is a test."
    result = engine.synthesize(text=text, voice_id=None, include_viseme=True)
    print("Audio file:", result.audio_filepath)
    print("Viseme events:")
    for evt in result.viseme_events:
        print(f"  {evt.timestamp_ms} ms -> {evt.viseme_id}")

if __name__ == "__main__":
    main()