"""
Test script for viseme synchronization
Uses phrases with very obvious mouth movements to verify timing
"""

import sys
import os

# Make sure we can import the orchestrator
sys.path.insert(0, os.path.dirname(__file__))

from conversation_orchestrator import BloomOrchestrator

def main():
    print("Viseme sync test")
    print("-" * 40)
    print("This script tests if mouth movements match audio")
    print()
    
    # Initialize orchestrator (opens face)
    print("Initializing orchestrator...")
    orchestrator = BloomOrchestrator()
    
    # Test phrases with very obvious mouth shapes
    test_phrases = [
        {
            "text": "Ooh ooh ooh", 
            "watch_for": "Mouth should make ROUND circle on 'ooh'"
        },
        {
            "text": "Ooh. ooh. ooh.", 
            "watch_for": "Mouth should make ROUND circle on 'ooh' and close between utterances"
        },
        {
            "text": "Ahhh ahhh ahhh",
            "watch_for": "Mouth should OPEN WIDE on 'ahhh'"
        },
        {
            "text": "Ahhh. ahhh. ahhh.",
            "watch_for": "Mouth should OPEN WIDE on 'ahhh' and close between utterances"
        },
        {
            "text": "Eee eee eee",
            "watch_for": "Mouth should make WIDE SMILE on 'eee'"
        },
        {
            "text": "Eee. eee. eee.",
            "watch_for": "Mouth should make WIDE SMILE on 'eee' and close between utterances"
        },
        {
            "text": "Boom boom boom",
            "watch_for": "Closed on 'b', round on 'oom'"
        },
        {
            "text": "Boom. boom. boom.",
            "watch_for": "Closed on 'b', round on 'oom'"
        },
        {
            "text": "See me say cheese",
            "watch_for": "Wide smile on 'see' and 'cheese'"
        },
        {
            "text": "Peter Piper picked a peck",
            "watch_for": "Mouth closes on each 'p' sound"
        },
        {
            "text": "One, two, three, four, five",
            "watch_for": "Different shapes for each number"
        }
    ]
    
    print("\n" + "=" * 40)
    print("VISEME SYNC TESTS")
    print("=" * 40)
    print()
    print("Watch the face carefully during each test.")
    print("The mouth should change shape to match the sounds.")
    print()
    
    for i, test in enumerate(test_phrases, 1):
        print(f"\nTest {i}/{len(test_phrases)}: \"{test['text']}\"")
        print(f"Watch for: {test['watch_for']}")
        
        input("Press Enter to play... ")
        
        orchestrator.speak(test['text'])
        
        response = input("Did the mouth sync correctly? (y/n/skip): ").strip().lower()
        
        if response == 'y':
            print("Good sync!")
        elif response == 'n':
            print("Sync issue noted")
        elif response == 'skip':
            print("Skipped")
        
        print()
    
    print("=" * 40)
    print("Testing complete!")
    print()
    print("If most tests passed, sync is good.")
    print("If many failed, may need to adjust timing delays.")

if __name__ == "__main__":
    main()