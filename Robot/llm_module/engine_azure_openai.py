import time
from typing import List, Dict, Optional

from openai import AzureOpenAI

from engine_interface import LLMEngineInterface, CompletionResult, CompletionMetrics


"""
Azure OpenAI implementation for the Bloom robot
Supports conversational AI with GPT models for speech therapy interactions
"""
class AzureOpenAIEngine(LLMEngineInterface):
    
    def __init__(
        self,
        api_key: str,
        endpoint: str,
        deployment_name: str,
        api_version: str = "2024-02-15-preview"  # Keep default for backwards compatibility
    ):
        self.api_key = api_key
        self.endpoint = endpoint
        self.deployment_name = deployment_name
        self.api_version = api_version
        
        # Initialize Azure OpenAI client
        self.client = AzureOpenAI(
            api_key=self.api_key,
            api_version=self.api_version,
            azure_endpoint=self.endpoint
        )
        
        # Default system prompt for Bloom robot
        self.default_system_prompt = (
            "You are Bloom, a friendly and helpful robot designed to assist with speech therapy. "
            "You are talking to children or individuals who may need extra patience and kindness. "
            "Keep your responses short, clear, and encouraging. Use simple language. "
            "Be supportive and celebrate their progress. Ask one question at a time."
        )
    
    """
    Generate a response from the LLM given conversation context
    """
    def generate_response(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
        max_tokens: int = 500
    ) -> CompletionResult:
        
        t_start = time.time()
        
        try:
            # Ensure system prompt is included if not already present
            if not messages or messages[0].get("role") != "system":
                messages = [{"role": "system", "content": self.default_system_prompt}] + messages
            
            # Call Azure OpenAI API
            response = self.client.chat.completions.create(
                model=self.deployment_name,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens
            )
            
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            # Extract response text
            response_text = response.choices[0].message.content
            
            # Extract token usage
            usage = response.usage
            prompt_tokens = usage.prompt_tokens if usage else None
            completion_tokens = usage.completion_tokens if usage else None
            total_tokens = usage.total_tokens if usage else None
            
            metrics = CompletionMetrics(
                total_latency_ms=total_latency_ms,
                prompt_tokens=prompt_tokens,
                completion_tokens=completion_tokens,
                total_tokens=total_tokens,
                success=True,
                error_reason=None
            )
            
            return CompletionResult(
                response_text=response_text,
                metrics=metrics
            )
            
        except Exception as e:
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            metrics = CompletionMetrics(
                total_latency_ms=total_latency_ms,
                prompt_tokens=None,
                completion_tokens=None,
                total_tokens=None,
                success=False,
                error_reason=str(e)
            )
            
            return CompletionResult(
                response_text=None,
                metrics=metrics
            )
    
    """
    Update the default system prompt for different contexts
    """
    def set_system_prompt(self, prompt: str):
        self.default_system_prompt = prompt