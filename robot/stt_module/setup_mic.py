"""
Microphone setup utility for Bloom STT module
Helps users select the correct microphone and test audio recording
"""

import sounddevice as sd
import soundfile as sf
import numpy as np

"""
List all available audio input devices
"""
def list_audio_devices():
    
    print("\n" + "="*60)
    print("Available audio input devices")
    print("="*60)
    
    devices = sd.query_devices()
    input_devices = []
    
    for i, device in enumerate(devices):
        if device['max_input_channels'] > 0:
            is_default = ' (DEFAULT)' if i == sd.default.device[0] else ''
            print(f"  [{i}] {device['name']}{is_default}")
            print(f"      Channels: {device['max_input_channels']}, Sample Rate: {device['default_samplerate']}")
            input_devices.append(i)
    
    print("="*60)
    return input_devices

"""
Test recording from a specific device
"""
def test_device(device_id, duration=3):
    
    print(f"\nTesting device {device_id}...")
    print(f"Recording for {duration} seconds - speak now!")
    print("-" * 60)
    
    try:
        audio = sd.rec(
            int(duration * 16000),
            samplerate=16000,
            channels=1,
            device=device_id,
            dtype='float32'
        )
        sd.wait()
        
        max_vol = np.max(np.abs(audio))
        print(f"Recording complete!")
        print(f"  Max volume: {max_vol:.4f}")
        
        if max_vol > 0.01:
            print(f"  Audio detected - this device is working!")
            
            # Save test file
            test_file = f"mic_test_device_{device_id}.wav"
            sf.write(test_file, audio, 16000, subtype='PCM_16')
            print(f"  Saved test recording to: {test_file}")
            print(f"  Play it to verify: afplay {test_file}")
            return True
        else:
            print(f"  No audio detected - try speaking louder or check permissions")
            return False
            
    except Exception as e:
        print(f"  Error: {e}")
        return False

"""
Save the selected device to config
"""
def save_device_preference(device_id):
    
    try:
        # Read current config
        with open('config.py', 'r') as f:
            lines = f.readlines()
        
        # Check if MICROPHONE_DEVICE already exists
        device_line_exists = False
        for i, line in enumerate(lines):
            if line.startswith('MICROPHONE_DEVICE'):
                lines[i] = f'MICROPHONE_DEVICE = {device_id}\n'
                device_line_exists = True
                break
        
        # Add it if it doesn't exist
        if not device_line_exists:
            lines.append(f'\n# Selected microphone device\nMICROPHONE_DEVICE = {device_id}\n')
        
        # Write back
        with open('config.py', 'w') as f:
            f.writelines(lines)
        
        print(f"\nSaved device {device_id} to config.py")
        return True
        
    except Exception as e:
        print(f"\nCould not save to config.py: {e}")
        print(f"  Manually add this line to config.py:")
        print(f"  MICROPHONE_DEVICE = {device_id}")
        return False

def main():
    print("\n" + "="*60)
    print("Bloom robot - microphone setup")
    print("="*60)
    print("\nThis utility helps you:")
    print("  1. Find available microphones")
    print("  2. Test which one works")
    print("  3. Save your preference")
    print()
    
    # List devices
    input_devices = list_audio_devices()
    
    if not input_devices:
        print("\nNo input devices found!")
        print("  Check your microphone connection and system settings")
        return
    
    # Get default device
    default_device = sd.default.device[0]
    
    print(f"\nDefault input device: {default_device}")
    print(f"Device name: {sd.query_devices(default_device)['name']}")
    
    # Ask if user wants to test default or select different
    print("\nOptions:")
    print("  [Enter] - test default device and continue")
    print("  [device number] - test a specific device")
    print("  [q] - quit")
    
    choice = input("\nYour choice: ").strip()
    
    if choice.lower() == 'q':
        print("Exiting...")
        return
    
    # Determine which device to test
    if choice == '':
        device_to_test = default_device
    else:
        try:
            device_to_test = int(choice)
            if device_to_test not in input_devices:
                print(f"Invalid device number: {device_to_test}")
                return
        except ValueError:
            print(f"Invalid input: {choice}")
            return
    
    # Test the device
    success = test_device(device_to_test, duration=3)
    
    if success:
        save_choice = input("\nSave this device to config.py? (y/n): ").strip().lower()
        if save_choice == 'y':
            save_device_preference(device_to_test)
            print("\nSetup complete! You can now run test_engine.py")
        else:
            print(f"\nTo use this device, update your code with: input_device={device_to_test}")
    else:
        print("\nDevice test failed. Try:")
        print("  - check System Settings -> Privacy & Security -> Microphone")
        print("  - increase microphone input volume")
        print("  - try a different device")
        print(f"  - run this script again with a different device number")

if __name__ == "__main__":
    main()