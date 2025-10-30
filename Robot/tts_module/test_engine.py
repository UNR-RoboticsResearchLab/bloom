from engine_azure import AzureTTSEngine

def main():
    engine = AzureTTSEngine(
        subscription_key="ABzgwVsFMDhbluo2oCA9GiirBbBnfToqaAfXOg8Bk6PIf6mIQjFKJQQJ99BJAC8vTInXJ3w3AAAYACOGfYLa",
        region="westus2",
        default_voice="en-US-AvaNeural"
    )
    text = "Hello robot, this is a test."
    result = engine.synthesize(text=text, voice_id=None, include_viseme=True)
    print("Audio file:", result.audio_filepath)
    print("Viseme events:")
    for evt in result.viseme_events:
        print(f"  {evt.timestamp_ms} ms → {evt.viseme_id}")

if __name__ == "__main__":
    main()
