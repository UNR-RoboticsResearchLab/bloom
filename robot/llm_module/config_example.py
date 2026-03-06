# Configuration file for API keys and secrets
# Copy this to config.py and add your actual keys

# Azure OpenAI credentials (required for default LLM engine)
AZURE_OPENAI_KEY = "your-azure-openai-key"
AZURE_OPENAI_ENDPOINT = "https://your-resource.openai.azure.com/"
AZURE_OPENAI_DEPLOYMENT = "gpt-4.1-mini"  # Your deployment name in Azure
AZURE_OPENAI_API_VERSION = "2024-02-15-preview"  # Check your Azure deployment for the correct version

# OpenAI credentials (optional, only needed if using OpenAI API directly)
OPENAI_API_KEY = "your-openai-api-key"
OPENAI_MODEL = "gpt-4.1-mini"