# Interface for LLM engines so we can swap between different providers if needed
# Example: if we need lower latency, different capabilities, or want to test local models

from abc import ABC, abstractmethod
from typing import Optional, List, Dict

class CompletionMetrics:
    def __init__(
        self,
        total_latency_ms: float,
        prompt_tokens: Optional[int],
        completion_tokens: Optional[int],
        total_tokens: Optional[int],
        success: bool,
        error_reason: Optional[str]
    ):
        self.total_latency_ms = total_latency_ms
        self.prompt_tokens = prompt_tokens
        self.completion_tokens = completion_tokens
        self.total_tokens = total_tokens
        self.success = success
        self.error_reason = error_reason

"""
response_text: The generated text response from the LLM (or None if failed)
metrics: CompletionMetrics object containing performance and status data
"""
class CompletionResult:
    def __init__(
        self,
        response_text: Optional[str],
        metrics: CompletionMetrics
    ):
        self.response_text = response_text
        self.metrics = metrics

class LLMEngineInterface(ABC):
    """
    Generate a response from the LLM given conversation context
    Args:
        messages: List of message dicts with 'role' and 'content' keys
                  Example: [{"role": "system", "content": "You are a helpful robot"},
                           {"role": "user", "content": "Hello!"}]
        temperature: Sampling temperature (0.0 to 2.0, higher = more random)
        max_tokens: Maximum tokens to generate in response
    Returns:
        CompletionResult object containing:
          - response_text: generated text (or None if failed)
          - metrics: CompletionMetrics with latency, token usage, success/failure, etc
    """
    @abstractmethod
    def generate_response(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
        max_tokens: int = 500
    ) -> CompletionResult:
        pass