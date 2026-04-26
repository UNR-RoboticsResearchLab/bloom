import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from llm_module.engine_azure_openai import AzureOpenAIEngine


class TestAzureOpenAIEngine(unittest.TestCase):

    def setUp(self):
        self.engine = AzureOpenAIEngine(
            api_key="fake-key",
            endpoint="https://fake.openai.azure.com/",
            deployment_name="gpt-4.1-mini"
        )

    def test_generate_response_returns_text(self):
        mock_client = MagicMock()
        mock_choice = MagicMock()
        mock_choice.message.content = "Hi! My name is Bloom. What is your name?"
        mock_response = MagicMock()
        mock_response.choices = [mock_choice]
        mock_response.usage.prompt_tokens = 10
        mock_response.usage.completion_tokens = 12
        mock_response.usage.total_tokens = 22
        mock_client.chat.completions.create.return_value = mock_response

        self.engine.client = mock_client

        messages = [{"role": "user", "content": "Hi Bloom, what is your name?"}]
        result = self.engine.generate_response(messages=messages)

        self.assertTrue(result.metrics.success)
        self.assertIsNotNone(result.response_text)
        self.assertIn("Bloom", result.response_text)

    def test_generate_response_handles_error(self):
        mock_client = MagicMock()
        mock_client.chat.completions.create.side_effect = Exception("API error")

        self.engine.client = mock_client

        messages = [{"role": "user", "content": "Hello"}]
        result = self.engine.generate_response(messages=messages)

        self.assertFalse(result.metrics.success)
        self.assertIsNone(result.response_text)
        self.assertIsNotNone(result.metrics.error_reason)


if __name__ == "__main__":
    unittest.main()