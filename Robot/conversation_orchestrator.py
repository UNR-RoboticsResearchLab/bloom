"""
Conversation orchestrator for Bloom robot
Manages conversation flow: STT -> LLM -> TTS -> Face
"""

import os
import sys
import time
import json
import webbrowser
import pygame
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading

# Add module paths
sys.path.append(os.path.join(os.path.dirname(__file__), 'stt_module'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'tts_module'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'llm_module'))

from stt_module.engine_azure import AzureSTTEngine
from tts_module.engine_azure import AzureTTSEngine
from llm_module.engine_azure_openai import AzureOpenAIEngine

class BloomOrchestrator:
    def __init__(self):
        print("Initializing bloom orchestrator")
        
        # Import configs from each module
        from stt_module import config as stt_config
        from tts_module import config as tts_config
        from llm_module import config as llm_config
        
        # System prompt for Bloom's personality
        self.system_prompt = """You are Bloom, a friendly robot helper designed to have conversations with children. You are patient, warm, and genuinely interested in what they have to say.

        Your conversation style:
        - Ask open-ended questions to learn about them
        - When they share something specific (like "I like basketball"), ask follow-up questions about it ("What position do you play?" or "Who's your favorite player?")
        - Keep responses short (2-3 sentences max)
        - Be encouraging and supportive
        - Let the child lead - if they want to change topics, go with it
        - If they seem stuck, gently help with a new question
        - Sound natural and conversational, not robotic

        Be curious about their interests, hobbies, and experiences."""

        # Initialize engines with credentials
        self.stt_engine = AzureSTTEngine(
            subscription_key=stt_config.AZURE_SUBSCRIPTION_KEY,
            region=stt_config.AZURE_REGION
        )
        self.tts_engine = AzureTTSEngine(
            subscription_key=tts_config.AZURE_SUBSCRIPTION_KEY,
            region=tts_config.AZURE_REGION
        )
        self.llm_engine = AzureOpenAIEngine(
            api_key=llm_config.AZURE_OPENAI_KEY,
            endpoint=llm_config.AZURE_OPENAI_ENDPOINT,
            deployment_name=llm_config.AZURE_OPENAI_DEPLOYMENT
        )

        # Set Bloom's personality
        self.llm_engine.set_system_prompt(self.system_prompt)
        
        # Conversation history (LLM needs this)
        self.conversation = []
        
        
        
        # Face control files
        self.face_dir = os.path.join(os.path.dirname(__file__), 'face_display')
        self.control_file = os.path.join(self.face_dir, 'face_control.json')
        
        # Initialize pygame for audio
        pygame.mixer.init()
        
        # Start HTTP server for face files
        self.start_http_server()
        
        # Open face in browser
        self.open_face()
        
        print("Orchestrator ready")
    
    def start_http_server(self):
         # Start simple HTTP server so face can access files using AJAX
        os.chdir(self.face_dir)
        
        # Create quiet version of handler (no logging)
        class QuietHandler(SimpleHTTPRequestHandler):
            def log_message(self, format, *args):
                # Suppress all HTTP request logs
                pass
        
        server = HTTPServer(('localhost', 8000), QuietHandler)
        
        # Run server in background thread
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        
        print("Started HTTP server on localhost:8000")
    
    def open_face(self):
        # Open face via HTTP server 
        face_url = "http://localhost:8000/blossom_face.html"
        webbrowser.open(face_url)
        print("Opened face in browser")
        
        # Wait for browser to load
        time.sleep(3)
    
    def set_face_emotion(self, emotion):
        # Write emotion command to control file
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'set_emotion',
                    'emotion': emotion,
                    'timestamp': time.time()
                }, f)
            print(f"Face emotion: {emotion}")
        except Exception as e:
            print(f"Could not set face emotion: {e}")
    
    def set_face_mode(self, mode):
        # Set face mouth mode (emotion or viseme)
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'set_mode',
                    'mode': mode,
                    'timestamp': time.time()
                }, f)
            print(f"Face mode: {mode}")
        except Exception as e:
            print(f"Could not set face mode: {e}")
    
    def send_visemes(self, visemes):
        # Write viseme timeline to control file
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'set_visemes',
                    'visemes': visemes,
                    'timestamp': time.time()
                }, f)
            print(f"Sent {len(visemes)} visemes to face")
        except Exception as e:
            print(f"Could not send visemes: {e}")
    
    def start_audio_sync(self):
        # Tell face to start audio sync
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'start_audio_sync',
                    'timestamp': time.time()
                }, f)
        except Exception as e:
            print(f"Could not start audio sync: {e}")
    
    def stop_audio_sync(self):
        # Tell face to stop audio sync
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'stop_audio_sync',
                    'timestamp': time.time()
                }, f)
        except Exception as e:
            print(f"Could not stop audio sync: {e}")

    def start_recording_timer(self, duration_seconds):
        # Tell face to show recording timer
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'start_recording',
                    'duration': duration_seconds,
                    'timestamp': time.time()
                }, f)
        except Exception as e:
            print(f"Could not start recording timer: {e}")

    def stop_recording_timer(self):
        # Tell face to hide recording timer
        try:
            with open(self.control_file, 'w') as f:
                json.dump({
                    'command': 'stop_recording',
                    'timestamp': time.time()
                }, f)
        except Exception as e:
            print(f"Could not stop recording timer: {e}")

    def listen(self):
        # Listen to user speech
        print("\nListening...")
        self.set_face_emotion('calm')
        
        # Show recording timer on face
        duration = 5.0
        self.start_recording_timer(duration)

        result = self.stt_engine.transcribe_from_mic(duration_seconds=5.0)
        
        # Hide recording timer
        self.stop_recording_timer()

        if result.metrics.success and result.text:
            print(f"You said: {result.text}")
            return result.text
        else:
            error = result.metrics.error_reason if result.metrics.error_reason else "Unknown error"
            print(f"Could not understand: {error}")
            return None
    
    def think(self, user_input):
        # Process with LLM
        print("Thinking...")
        self.set_face_emotion('thinking')
        
        # Add user message to conversation
        self.conversation.append({
            "role": "user",
            "content": user_input
        })
        
        # Get LLM response
        result = self.llm_engine.generate_response(
            messages=self.conversation
        )
        
        if result.metrics.success:
            response_text = result.response_text
            print(f"Bloom: {response_text}")
            
            # Add to conversation history
            self.conversation.append({
                "role": "assistant",
                "content": response_text
            })
            
            return response_text
        else:
            # Handle error or filtered content
            if not result.metrics.success:
                error_msg = result.metrics.error_reason
                if 'content_filter' in str(error_msg):
                    print("LLM error: Content filtered by Azure safety policy")
                else:
                    print(f"LLM error: {error_msg}")
            else:
                print("LLM returned empty response (possibly content filtered)")
            
            # Return fallback response
            fallback = "I'm sorry, I had trouble thinking of what to say. Can you say that differently?"
            
            # Add fallback to conversation history
            self.conversation.append({
                "role": "assistant",
                "content": fallback
            })
            
            return fallback
    
    def speak(self, text):
        # Speak text with TTS and show on face
        print(f"Speaking: {text}")
        self.set_face_emotion('happy')
        
        # Generate speech with visemes
        result = self.tts_engine.synthesize(
            text=text,
            include_viseme=True
        )
        
        # Check if synthesis succeeded
        if not result.metrics.success:
            print(f"TTS error: {result.metrics.error_reason}")
            return
        
        # Get audio file path
        audio_file = result.audio_filepath
        
        # Send viseme data to face (prepare early)
        if result.viseme_events:
            visemes = [
                {
                    "viseme_id": int(event.viseme_id),
                    "audio_offset": event.timestamp_ms * 10000
                }
                for event in result.viseme_events
            ]
            print(f"Got {len(visemes)} visemes")
            self.send_visemes(visemes)
            time.sleep(0.1)  # Brief moment for face to receive
            self.set_face_mode('viseme')
            time.sleep(0.1)  # Brief moment to switch mode

        # Play audio - start sync at exact moment
        try:
            pygame.mixer.music.load(audio_file)
            
            # Start audio and face sync simultaneously
            if result.viseme_events:
                self.start_audio_sync()
                time.sleep(0.05)  # Just 50ms for command to reach face
            
            pygame.mixer.music.play()
            
            # Wait for audio to finish
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
            
            # Stop viseme sync
            self.stop_audio_sync()
            self.set_face_mode('emotion')
            
            # Clean up
            pygame.mixer.music.unload()
            if os.path.exists(audio_file):
                os.remove(audio_file)
        
        except Exception as e:
            print(f"Audio playback error: {e}")
    
    def introduce(self):
        # Bloom introduces itself
        intro = "Hi! I'm Bloom, your friendly robot helper. I'd love to get to know you! What's your name?"
        self.speak(intro)
        
        # Add to conversation history
        self.conversation.append({
            "role": "assistant",
            "content": intro
        })
    
    def run_conversation(self):
        # Main conversation loop
        print("\n" + "=" * 40)
        print("Starting conversation")
        print("Press Ctrl+C to stop")
        print("=" * 40)
        
        # Bloom introduces itself
        self.introduce()
        
        try:
            while True:
                # Listen to user
                user_input = self.listen()
                
                if not user_input:
                    print("Trying again")
                    continue
                
                # Get response from LLM
                response = self.think(user_input)
                
                # Speak response
                self.speak(response)
                
                # Small pause
                time.sleep(0.5)
        
        except KeyboardInterrupt:
            print("\n\nEnding conversation")
            self.speak("Goodbye! It was really nice talking to you!")

def main():
    print("Bloom conversation orchestrator")
    print("-" * 40)
    
    # Create orchestrator
    orchestrator = BloomOrchestrator()
    
    # Run conversation
    orchestrator.run_conversation()

if __name__ == "__main__":
    main()