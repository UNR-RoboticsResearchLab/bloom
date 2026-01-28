# LLM Module

Large Language Model integration for Bloom robot conversations

## Setup

### 1. Install dependencies
```bash
cd llm_module
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Configure API keys
```bash
cp config_example.py config.py
```

Edit `config.py` with your credentials:
- Get Azure OpenAI keys from Azure AI Foundry
- Get OpenAI API keys from https://platform.openai.com/api-keys

### 3. Test
```bash
python test_engine.py
```

## Engines

**Azure OpenAI (default):** Uses Azure student credits, GPT-4.1-mini
**OpenAI API (alternative):** Direct OpenAI API access

Switch engines in `test_engine.py` by commenting/uncommenting the ENGINE initialization.

## Basic Usage
```python
from engine_azure_openai import AzureOpenAIEngine

engine = AzureOpenAIEngine(
    api_key="your-key",
    endpoint="https://your-project.openai.azure.com/",
    deployment_name="gpt-4.1-mini",
    api_version="2024-02-15-preview"
)

# Single message
messages = [{"role": "user", "content": "Hello!"}]
result = engine.generate_response(messages=messages)

print(result.response_text)
print(f"Latency: {result.metrics.total_latency_ms}ms")
print(f"Tokens: {result.metrics.total_tokens}")
```

## Conversation Context

LLMs are stateless - maintain conversation history yourself:
```python
conversation = []

# User message
conversation.append({"role": "user", "content": "My name is Sam"})
result = engine.generate_response(messages=conversation)

# Add response to history
conversation.append({"role": "assistant", "content": result.response_text})

# Next message (will remember "Sam")
conversation.append({"role": "user", "content": "What's my name?"})
result = engine.generate_response(messages=conversation)
```

## Custom System Prompts
```python
custom_prompt = "You are a helpful robot. Be encouraging and patient."
engine.set_system_prompt(custom_prompt)
```

## Parameters

**temperature (0.0-2.0):**
- 0.0-0.3: Predictable
- 0.5-0.7: Balanced (default)
- 0.8+: Creative

**max_tokens:**
- 100-200: Short (1-2 sentences)
- 300-500: Medium (default)
- 500+: Long

## Metrics

Result includes:
- `response_text` - Generated text
- `metrics.total_latency_ms` - Response time
- `metrics.prompt_tokens` - Input tokens
- `metrics.completion_tokens` - Output tokens
- `metrics.total_tokens` - Total (for cost calculation)
- `metrics.success` - Success flag
- `metrics.error_reason` - Error details if failed

## Files

- `engine_azure_openai.py` - Azure OpenAI implementation
- `engine_openai.py` - OpenAI API implementation
- `engine_interface.py` - Abstract interface
- `test_engine.py` - Test script
- `config_example.py` - Template config
- `requirements.txt` - Dependencies