# Configuration file for API keys and secrets
# Copy this to config.py and add your actual keys

# Azure Speech-to-Text credentials (required for default engine)
AZURE_SUBSCRIPTION_KEY = "your-azure-key-here"
AZURE_REGION = "westus2"

# OpenAI Whisper credentials (optional, only needed if using Whisper engine)
OPENAI_API_KEY = "your-openai-api-key"

# Microphone device (run setup_mic.py to find the right one)
# Set to None to use system default, or specify device number (e.g., 0, 1, 2)
MICROPHONE_DEVICE = None