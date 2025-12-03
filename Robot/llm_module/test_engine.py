from engine_azure_openai import AzureOpenAIEngine
from engine_openai import OpenAIEngine
from config import (
    AZURE_OPENAI_KEY, 
    AZURE_OPENAI_ENDPOINT, 
    AZURE_OPENAI_DEPLOYMENT,
    AZURE_OPENAI_API_VERSION,
    OPENAI_API_KEY,
    OPENAI_MODEL
)

# ===== CHOOSE YOUR ENGINE HERE =====

# Option 1: Azure OpenAI (uses Azure student credits)
ENGINE = AzureOpenAIEngine(
    api_key=AZURE_OPENAI_KEY,
    endpoint=AZURE_OPENAI_ENDPOINT,
    deployment_name=AZURE_OPENAI_DEPLOYMENT,
    api_version=AZURE_OPENAI_API_VERSION
)

# Option 2: OpenAI API (requires OpenAI credits)
# ENGINE = OpenAIEngine(
#     api_key=OPENAI_API_KEY,
#     model=OPENAI_MODEL
# )

# ===================================

"""
Test basic conversation with the LLM
"""
def test_conversation():
    
    print("\n" + "="*60)
    print("Testing Bloom robot conversation")
    print(f"Using engine: {ENGINE.__class__.__name__}")
    print("="*60)
    
    # Simulate a conversation
    conversation = [
        {"role": "user", "content": "Hello! My name is Tester."}
    ]
    
    print("\nUser: Hello! My name is Tester.")
    
    # Get response from LLM
    result = ENGINE.generate_response(
        messages=conversation,
        temperature=0.7,
        max_tokens=300
    )
    
    if result.metrics.success:
        print(f"\nBloom: {result.response_text}")
        
        print(f"\nMetrics:")
        print(f"  Total latency: {result.metrics.total_latency_ms:.2f} ms")
        print(f"  Prompt tokens: {result.metrics.prompt_tokens}")
        print(f"  Completion tokens: {result.metrics.completion_tokens}")
        print(f"  Total tokens: {result.metrics.total_tokens}")
        
        # Continue conversation
        conversation.append({"role": "assistant", "content": result.response_text})
        conversation.append({"role": "user", "content": "I like playing with robots!"})
        
        print("\n" + "-"*60)
        print("\nUser: I like playing with robots!")
        
        result2 = ENGINE.generate_response(
            messages=conversation,
            temperature=0.7,
            max_tokens=300
        )
        
        if result2.metrics.success:
            print(f"\nBloom: {result2.response_text}")
            
            print(f"\nMetrics:")
            print(f"  Total latency: {result2.metrics.total_latency_ms:.2f} ms")
            print(f"  Total tokens: {result2.metrics.total_tokens}")
        else:
            print(f"\nConversation failed: {result2.metrics.error_reason}")
    else:
        print(f"\nConversation failed: {result.metrics.error_reason}")
    
    print("\n" + "="*60)

"""
Test custom system prompt
"""
def test_custom_prompt():
    
    # Set a custom system prompt for a specific lesson
    custom_prompt = (
        "You are Bloom, a speech therapy robot. "
        "You are conducting a lesson on the /s/ sound. "
        "Ask the student to practice words that start with 's'. "
        "Be encouraging and patient."
    )
    
    ENGINE.set_system_prompt(custom_prompt)
    
    print("\n" + "="*60)
    print("Testing custom system prompt (/s/ sound lesson)")
    print(f"Using engine: {ENGINE.__class__.__name__}")
    print("="*60)
    
    conversation = [
        {"role": "user", "content": "I'm ready to practice!"}
    ]
    
    print("\nUser: I'm ready to practice!")
    
    result = ENGINE.generate_response(
        messages=conversation,
        temperature=0.6,
        max_tokens=200
    )
    
    if result.metrics.success:
        print(f"\nBloom: {result.response_text}")
        print(f"\nTokens used: {result.metrics.total_tokens}")
    else:
        print(f"\nLesson failed: {result.metrics.error_reason}")
    
    print("\n" + "="*60)

if __name__ == "__main__":
    # Uncomment the test you want to run:
    
    #test_conversation()
    test_custom_prompt()