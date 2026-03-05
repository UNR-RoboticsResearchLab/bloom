import os
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from llm_module.engine_azure_openai import AzureOpenAIEngine


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.get_logger().info('Initializing LLM node')

        llm_key = os.environ.get('AZURE_OPENAI_KEY', '')
        llm_endpoint = os.environ.get('AZURE_OPENAI_ENDPOINT', '')
        llm_deployment = os.environ.get('AZURE_OPENAI_DEPLOYMENT', 'gpt-4.1-mini')
        llm_api_version = os.environ.get('AZURE_OPENAI_API_VERSION', '2024-02-15-preview')

        if not llm_key or not llm_endpoint:
            self.get_logger().error('AZURE_OPENAI_KEY or AZURE_OPENAI_ENDPOINT not set - check your .env file')

        self.llm_engine = AzureOpenAIEngine(
            api_key=llm_key,
            endpoint=llm_endpoint,
            deployment_name=llm_deployment,
            api_version=llm_api_version
        )

        self.llm_engine.set_system_prompt("""You are Bloom, a friendly robot helper designed to have conversations with children. You are patient, warm, and genuinely interested in what they have to say.

Your conversation style:
- Ask open-ended questions to learn about them
- When they share something specific (like "I like basketball"), ask follow-up questions about it
- Keep responses short (2-3 sentences max)
- Be encouraging and supportive
- Let the child lead - if they want to change topics, go with it
- If they seem stuck, gently help with a new question
- Sound natural and conversational, not robotic

Be curious about their interests, hobbies, and experiences.""")

        self.conversation = []
        self.current_state = 'waiting'

        self.stt_sub = self.create_subscription(
            String, '/vosk/result', self.on_stt_result, 10)
        self.state_sub = self.create_subscription(
            String, 'robot/state', self.on_state_update, 10)

        self.tts_pub = self.create_publisher(String, '/tts/speak', 10)
        self.state_cmd_pub = self.create_publisher(String, 'robot/state_cmd', 10)

        self.get_logger().info('LLM node ready - listening on /vosk/result')

    def on_state_update(self, msg: String):
        self.current_state = msg.data

    def set_robot_state(self, state: str):
        msg = String()
        msg.data = state
        self.state_cmd_pub.publish(msg)
        self.get_logger().info(f'Robot state -> {state}')

    def on_stt_result(self, msg: String):
        if not msg.data:
            return
        if self.current_state in ('talking', 'loading'):
            self.get_logger().info(f'Ignoring STT input - robot is currently {self.current_state}')
            return
        thread = threading.Thread(target=self.think, args=(msg.data,), daemon=True)
        thread.start()

    def think(self, user_input: str):
        self.get_logger().info(f'User said: {user_input}')
        self.set_robot_state('loading')

        self.conversation.append({'role': 'user', 'content': user_input})

        result = self.llm_engine.generate_response(messages=self.conversation)

        if result.metrics.success and result.response_text:
            response_text = result.response_text
            self.get_logger().info(f'Bloom: {response_text}')
            self.conversation.append({'role': 'assistant', 'content': response_text})
        else:
            error = result.metrics.error_reason or 'Unknown error'
            if 'content_filter' in str(error):
                self.get_logger().warn('LLM response filtered by Azure content policy')
            else:
                self.get_logger().error(f'LLM error: {error}')
            response_text = "I'm sorry, I had trouble thinking of what to say. Can you say that differently?"
            self.conversation.append({'role': 'assistant', 'content': response_text})

        tts_msg = String()
        tts_msg.data = response_text
        self.tts_pub.publish(tts_msg)

    def clear_conversation(self):
        self.conversation = []
        self.get_logger().info('Conversation history cleared')


def main(args=None):
    env_path = os.path.join(get_package_share_directory('bloom_speech'), '.env')
    if os.path.exists(env_path):
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    os.environ.setdefault(key.strip(), value.strip())
    else:
        print(f'WARNING: .env not found at {env_path}')
        print('Copy .env.example to .env and fill in your credentials')

    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()