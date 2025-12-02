"""
Test script for the face display
Opens the face in a browser and shows how to test it manually
"""

import webbrowser
import os
import time
import json

# Get path to HTML file in same directory as this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_FILE = os.path.join(SCRIPT_DIR, "blossom_face.html")

def open_face():
    # Open face in browser
    print("Opening face in browser")
    
    # Windows uses backslashes
    file_path = HTML_FILE.replace('\\', '/')
    file_url = f"file:///{file_path}"
    
    webbrowser.open(file_url)
    print(f"Opened: {file_url}")

def show_controls():
    # Print keyboard controls for manual testing
    print("\nKeyboard controls:")
    print("  1-8: Change emotions (happy, sad, excited, etc)")
    print("  V: Switch to viseme mode")
    print("  E: Switch to emotion mode")
    print("  T: Test all 9 mouth shapes")
    print("  S: Show status (current mode and viseme)")
    print("  Arrow keys: Cycle through visemes (only works in viseme mode)")
    print("  Space: Blink")
    print("  C: Show/hide control panel")

def show_testing_checklist():
    # Print what to verify
    print("\nTesting checklist:")
    print("  - Press 2 (happy) - should smile")
    print("  - Press 3 (sad) - should frown")
    print("  - Press V then T - should see 9 different mouth shapes")
    print("  - Press V then Arrow Right - should cycle through visemes 0-21")
    print("  - All mouth shapes should look consistent (same style)")

def create_sample_viseme_data():
    # Create example viseme timeline (what our Azure Enging returns for visemes)
    visemes = [
        {"viseme_id": 0, "audio_offset": 0},
        {"viseme_id": 21, "audio_offset": 1000000},
        {"viseme_id": 4, "audio_offset": 1500000},
        {"viseme_id": 18, "audio_offset": 2500000},
        {"viseme_id": 14, "audio_offset": 3500000},
        {"viseme_id": 0, "audio_offset": 5000000}
    ]
    
    # Save to JSON file (this is how orchestrator will send data to face)
    json_file = os.path.join(SCRIPT_DIR, "current_visemes.json")
    with open(json_file, 'w') as f:
        json.dump({'visemes': visemes, 'timestamp': time.time()}, f)
    
    print(f"\nCreated example viseme file: {json_file}")
    print("Sample timeline for 'Hello':")
    for v in visemes:
        time_ms = v["audio_offset"] / 10000
        print(f"  {time_ms:.1f}ms - viseme {v['viseme_id']}")

def main():
    print("Face display test")
    print("-" * 40)
    
    # Check if HTML file exists
    if not os.path.exists(HTML_FILE):
        print(f"Error: {HTML_FILE} not found")
        print("Make sure blossom_face.html is in this directory")
        return
    
    print("Found HTML file")
    
    # Open face in browser
    open_face()
    
    # Wait a sec for browser to open
    time.sleep(1)
    
    # Show how to test it
    show_controls()
    show_testing_checklist()
    
    # Create sample viseme data
    create_sample_viseme_data()
    
    print("\nNext steps:")
    print("  1. Test the face manually with keyboard controls")
    print("  2. Build Python-JavaScript bridge to control from code")
    print("  3. Integrate with TTS module to get real viseme data")
    print("  4. Build orchestrator to tie everything together")

if __name__ == "__main__":
    main()