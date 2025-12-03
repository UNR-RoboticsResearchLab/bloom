# LLM Module

## Prerequisites:
- Python 3.x
- Azure OpenAI deployment (default engine)
- OpenAI API key (optional, for OpenAI engine)

This module provides Large Language Model (LLM) functionality for the Bloom robot. The system uses Azure OpenAI (GPT-4.1-mini) by default, but includes an interface to swap engines if needed (e.g., switching to OpenAI API directly or testing local models)

## Features:
- Azure OpenAI engine (default, uses Azure student credits)
- OpenAI API engine (alternative, requires OpenAI credits)
- Conversation context management (LLMs are stateless, so we maintain history)
- Customizable system prompts for different lesson types
- Returns performance metrics (latency, token usage, costs)
- LLMEngineInterface: allows LLM engine to be swapped out if needed

## Getting Started:

### 1. Install dependencies

Navigate to llm_module:
```bash
cd ~/bloom/Robot/llm_module
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
# Azure OpenAI credentials (required for default engine)
AZURE_OPENAI_KEY = "your-azure-openai-key"
AZURE_OPENAI_ENDPOINT = "https://your-project.openai.azure.com/"
AZURE_OPENAI_DEPLOYMENT = "gpt-4.1-mini"
AZURE_OPENAI_API_VERSION = "2024-02-15-preview"

# OpenAI credentials (optional, only needed if using OpenAI API)
OPENAI_API_KEY = "your-openai-api-key"
OPENAI_MODEL = "gpt-4.1-mini"
```

Get your Azure OpenAI credentials from: Azure AI Foundry -> Your Project -> Deployments

Get your OpenAI API key from: https://platform.openai.com/api-keys

### 3. Run the test script
```bash
python test_engine.py
```

This script should output:
- A simulated conversation with Bloom
- Performance metrics (latency, token usage)

## Switching Between Engines:

The module uses **Azure OpenAI by default**. To switch to the OpenAI API:

1. Open `test_engine.py`
2. Comment out the Azure engine and uncomment the OpenAI engine:
```python
# Option 1: Azure OpenAI (uses Azure student credits)
# ENGINE = AzureOpenAIEngine(
#     api_key=AZURE_OPENAI_KEY,
#     endpoint=AZURE_OPENAI_ENDPOINT,
#     deployment_name=AZURE_OPENAI_DEPLOYMENT,
#     api_version=AZURE_OPENAI_API_VERSION
# )

# Option 2: OpenAI API (requires OpenAI credits)
ENGINE = OpenAIEngine(
    api_key=OPENAI_API_KEY,
    model=OPENAI_MODEL
)
```

3. Make sure you've added your OpenAI API key to config.py

## Usage in Code:
```python
from engine_azure_openai import AzureOpenAIEngine

engine = AzureOpenAIEngine(
    api_key="YOUR_AZURE_KEY",
    endpoint="https://your-project.openai.azure.com/",
    deployment_name="gpt-4.1-mini",
    api_version="2024-02-15-preview"
)

# Simple conversation
messages = [
    {"role": "user", "content": "Hello! My name is Emma."}
]

result = engine.generate_response(
    messages=messages,
    temperature=0.7,
    max_tokens=300
)

print("Bloom:", result.response_text)
print("Metrics:")
print("  Total latency (ms):", result.metrics.total_latency_ms)
print("  Prompt tokens:", result.metrics.prompt_tokens)
print("  Completion tokens:", result.metrics.completion_tokens)
print("  Total tokens:", result.metrics.total_tokens)
print("  Success:", result.metrics.success)
if not result.metrics.success:
    print("  Error reason:", result.metrics.error_reason)

# Continue the conversation (maintaining context)
messages.append({"role": "assistant", "content": result.response_text})
messages.append({"role": "user", "content": "I like drawing!"})

result2 = engine.generate_response(messages=messages, temperature=0.7, max_tokens=300)
print("Bloom:", result2.response_text)
```

### Customizing System Prompts:

For different lesson types, you can customize the robot's behavior:
```python
# For a specific speech therapy lesson
custom_prompt = (
    "You are Bloom, a speech therapy robot. "
    "You are conducting a lesson on the /s/ sound. "
    "Ask the student to practice words that start with 's'. "
    "Be encouraging and patient."
)

engine.set_system_prompt(custom_prompt)

# Now all responses will follow this lesson context
result = engine.generate_response(
    messages=[{"role": "user", "content": "I'm ready!"}],
    temperature=0.6,
    max_tokens=200
)
```

## Performance Metrics:

The module returns performance data with each completion request, allowing for metrics tracking and cost monitoring

- **total_latency_ms** - time in milliseconds from when request was sent to when response is ready
- **prompt_tokens** - number of tokens in the input (conversation history + system prompt)
- **completion_tokens** - number of tokens in the generated response
- **total_tokens** - sum of prompt and completion tokens (used for cost calculation)
- **success** - boolean flag indicating if generation succeeded
- **error_reason** - string describing the failure reason (if success = false)

## Understanding Parameters:

### temperature (0.0 to 2.0)

Controls response randomness/creativity:
- **0.0-0.3** - Very consistent, predictable responses (good for scripted lessons)
- **0.5-0.7** - Balanced, natural conversation (recommended default)
- **0.8-1.5** - More creative, varied responses (use carefully with children)
- **1.5-2.0** - Very random (not recommended for therapy)

### max_tokens

Limits response length:
- **100-200** - Short responses (1-2 sentences)
- **300-500** - Medium responses (2-4 sentences, recommended)
- **500+** - Longer responses (use sparingly to avoid robot monologues)

Note: 1 token is approximately 0.75 words

## Conversation Context Management:

LLMs are stateless - they don't remember previous messages unless you provide them. Always maintain conversation history:
```python
# Initialize conversation
conversation = []

# User says something
user_message = {"role": "user", "content": "My name is Sam"}
conversation.append(user_message)

# Get response
result = engine.generate_response(messages=conversation)

# Add Bloom's response to history
conversation.append({"role": "assistant", "content": result.response_text})

# User says something else
user_message2 = {"role": "user", "content": "What's my name?"}
conversation.append(user_message2)

# Bloom will remember because we included the full history
result2 = engine.generate_response(messages=conversation)
# Response will reference "Sam" from earlier
```

## Technical Details:

- **Model:** GPT-4.1-mini (fast, cost-effective, high quality)
- **API version:** 2024-02-15-preview (Azure)
- **Default system prompt:** Defines Bloom as friendly, patient, child-appropriate robot
- **Message format:** OpenAI chat format with role/content structure
- **Context window:** Up to 128K tokens (practically unlimited for conversations)