import evdev
from evdev import categorize, ecodes

# Replace 'event3' with the event number you found above
# Use ls -l /dev/input/by-id/ to find correct input event device for your keyboard. It may be different on your system.
device_path = '/dev/input/event4'

try:
    dev = evdev.InputDevice(device_path)
    print(f"Listening for inputs on: {dev.name}...")
except PermissionError:
    print(f"Permission denied. Run with sudo, or add your user to the 'input' group.")
    exit(1)

# Start an infinite loop that yields events as they happen
for event in dev.read_loop():
    
    # Filter for key events (ignore mouse movements, sync events, etc.)
    if event.type == ecodes.EV_KEY:
        
        # categorize() translates raw electrical events into human-readable KeyEvents
        key_event = categorize(event)
        
        # Only print when the key is pressed DOWN (ignore key releases and holds)
        if key_event.keystate == key_event.key_down:
            print(f"You typed: {key_event.keycode}")