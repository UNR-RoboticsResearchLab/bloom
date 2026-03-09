import os
import sys
_robot_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', '..', '..', '..', '..', 'bloom', 'Robot'))
if _robot_dir not in sys.path:
    sys.path.insert(0, _robot_dir)
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from llm_module.engine_azure_openai import AzureOpenAIEngine

FREE_CONVERSATION_PROMPT = """You are Bloom, a friendly robot helper designed to have conversations with children. You are patient, warm, and genuinely interested in what they have to say.

Your conversation style:
- Ask open-ended questions to learn about them
- When they share something specific (like "I like basketball"), ask follow-up questions about it
- Keep responses short (2-3 sentences max)
- Be encouraging and supportive
- Let the child lead - if they want to change topics, go with it
- If they seem stuck, gently help with a new question
- Sound natural and conversational, not robotic

Be curious about their interests, hobbies, and experiences."""

LESSON_TANGENT_PROMPT_TEMPLATE = """You are Bloom, a friendly robot teacher helping a child during a lesson. You are currently handling a tangent question from the student.

Lesson context: {context}

Your job during this tangent:
- Answer the student's question naturally and warmly in 2-3 sentences
- Stay focused on the lesson topic
- Be encouraging and supportive
- After answering, ALWAYS ask "Does that make sense? Do you have any more questions about that?"
- When the student indicates they are satisfied (says yes, okay, no more questions, thanks, got it, etc.), respond with a brief encouraging sentence and include the exact token [RETURN_TO_LESSON] at the very end
- If the student asks another follow-up question, answer it, then ask again if they have more questions
- IMPORTANT: You must eventually include [RETURN_TO_LESSON] once the student is done. Never leave the tangent open-ended."""

SINGLE_TURN_PROMPT_TEMPLATE = """You are Bloom, a friendly robot teacher. You are evaluating a student's response to a question during a lesson.

{prompt}

Important rules:
- Keep your response to 2-3 sentences maximum
- Be warm, encouraging, and age-appropriate
- Always end your response with the exact token [LESSON_CONTINUE] on its own
- Do not ask follow-up questions or invite more responses"""

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

        self.llm_engine.set_system_prompt(FREE_CONVERSATION_PROMPT)

        self.conversation = []
        self.current_state = 'waiting'
        self.mode = 'free_conversation'
        self.lesson_context = ''

        self.stt_sub = self.create_subscription(
            String, '/vosk/result', self.on_stt_result, 10)
        self.state_sub = self.create_subscription(
            String, 'robot/state', self.on_state_update, 10)
        self.mode_sub = self.create_subscription(
            String, '/llm/mode', self.on_mode_update, 10)
        self.context_sub = self.create_subscription(
            String, '/llm/lesson_context', self.on_lesson_context, 10)

        self.tts_pub = self.create_publisher(String, '/tts/speak', 10)
        self.state_cmd_pub = self.create_publisher(String, 'robot/state_cmd', 10)
        self.wrap_up_pub = self.create_publisher(String, '/llm/wrap_up', 10)

        self.get_logger().info('LLM node ready - listening on /vosk/result')
        self.single_turn_consumed = False

    def on_state_update(self, msg: String):
        self.current_state = msg.data

    def on_mode_update(self, msg: String):
        new_mode = msg.data
        if new_mode != self.mode:
            self.get_logger().info(f'[LLM_MODE] {self.mode} -> {new_mode}, history cleared')
            self.mode = new_mode
            self.conversation = []
            if new_mode == 'lesson_tangent' and self.lesson_context:
                prompt = LESSON_TANGENT_PROMPT_TEMPLATE.format(context=self.lesson_context)
                self.llm_engine.set_system_prompt(prompt)
            elif new_mode == 'single_turn' and self.lesson_context:
                prompt = SINGLE_TURN_PROMPT_TEMPLATE.format(prompt=self.lesson_context)
                self.llm_engine.set_system_prompt(prompt)
                self.single_turn_consumed = False
            elif new_mode == 'lesson_mode':
                self.lesson_context = ''
                self.llm_engine.set_system_prompt(FREE_CONVERSATION_PROMPT)
            else:
                self.llm_engine.set_system_prompt(FREE_CONVERSATION_PROMPT)

    def on_lesson_context(self, msg: String):
        self.lesson_context = msg.data
        self.get_logger().info(f'Lesson context updated: {self.lesson_context}')
        if self.mode == 'lesson_tangent':
            prompt = LESSON_TANGENT_PROMPT_TEMPLATE.format(context=self.lesson_context)
            self.llm_engine.set_system_prompt(prompt)

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
        if self.mode == 'lesson_mode':
            self.get_logger().info('Ignoring STT input - lesson is active, not in tangent')
            return
        if self.mode == 'single_turn' and self.single_turn_consumed:
            self.get_logger().info('Ignoring STT input - single turn already consumed')
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

        return_to_lesson = False
        lesson_continue = False

        if self.mode == 'lesson_tangent' and '[RETURN_TO_LESSON]' in response_text:
            response_text = response_text.replace('[RETURN_TO_LESSON]', '').strip()
            self.mode = 'lesson_mode'
            self.get_logger().info('[LLM] RETURN_TO_LESSON detected')
            return_to_lesson = True
        elif self.mode == 'single_turn' and '[LESSON_CONTINUE]' in response_text:
            response_text = response_text.replace('[LESSON_CONTINUE]', '').strip()
            self.single_turn_consumed = True
            self.mode = 'lesson_mode'
            self.get_logger().info('[LLM] LESSON_CONTINUE detected')
            lesson_continue = True

        tts_msg = String()
        tts_msg.data = response_text
        self.tts_pub.publish(tts_msg)

        if return_to_lesson:
            wrap_msg = String()
            wrap_msg.data = 'done'
            self.wrap_up_pub.publish(wrap_msg)
        elif lesson_continue:
            wrap_msg = String()
            wrap_msg.data = 'continue'
            self.wrap_up_pub.publish(wrap_msg)

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