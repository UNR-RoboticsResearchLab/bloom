"""Speech engines shared by the Bloom robot's ROS 2 nodes (bloom_speech) and the standalone demo.

Plain Python with no ROS imports, so it works with or without a ROS install:
    bloom_speech_engines.tts  - text to speech with visemes (Azure)
    bloom_speech_engines.llm  - conversational LLM (Azure OpenAI, OpenAI)

Engines take their credentials as constructor arguments and never read config files or the
environment themselves; callers decide where credentials come from.
"""
