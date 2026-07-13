from pyorbbecsdk import *
import cv2
import numpy as np
import time

print("Creating pipeline...")
pipeline = Pipeline()
device = pipeline.get_device()

if not device:
    print("No device!")
    exit()

print(f"Device: {device.get_device_info().get_name()}")

# Configure streams
config = Config()

# Get color sensor and its profiles
color_sensor = device.get_sensor(OBSensorType.COLOR_SENSOR)
if color_sensor:
    profile_list = color_sensor.get_stream_profile_list()
    print(f"Color profiles available: {len(profile_list)}")
    
    # List all profiles for debugging
    for i in range(len(profile_list)):
        p = profile_list.get_profile(i)
        print(f"  {i}: {p.get_width()}x{p.get_height()} @ {p.get_fps()}fps (format: {p.get_format()})")
    
    # Try each profile until one works
    color_enabled = False
    for i in range(len(profile_list)):
        try:
            profile = profile_list.get_profile(i)
            config.enable_stream(profile)
            print(f"Enabled color: {profile.get_width()}x{profile.get_height()} @ {profile.get_fps()}fps (format: {profile.get_format()})")
            color_enabled = True
            break
        except Exception as e:
            print(f"Failed to enable profile {i}: {e}")
    
    if not color_enabled:
        print("❌ Could not enable any color profile")
else:
    print("❌ No color sensor found!")

# Get depth sensor and its profiles
depth_sensor = device.get_sensor(OBSensorType.DEPTH_SENSOR)
if depth_sensor:
    profile_list = depth_sensor.get_stream_profile_list()
    print(f"Depth profiles available: {len(profile_list)}")
    
    for i in range(len(profile_list)):
        p = profile_list.get_profile(i)
        print(f"  {i}: {p.get_width()}x{p.get_height()} @ {p.get_fps()}fps")
    
    # Try each profile
    depth_enabled = False
    for i in range(len(profile_list)):
        try:
            profile = profile_list.get_profile(i)
            config.enable_stream(profile)
            print(f"Enabled depth: {profile.get_width()}x{profile.get_height()} @ {profile.get_fps()}fps")
            depth_enabled = True
            break
        except Exception as e:
            print(f"Failed to enable depth profile {i}: {e}")
    
    if not depth_enabled:
        print("❌ Could not enable any depth profile")
else:
    print("❌ No depth sensor found!")

print("Starting pipeline...")
pipeline.start(config)
time.sleep(1)  # Give camera time to warm up

print("\nStreaming... Press 'q' or ESC to exit")
print("Trying to get frames...")

frame_count = 0
while frame_count < 30:  # Try for 30 frames
    frames = pipeline.wait_for_frames(200)
    if frames:
        color_frame = frames.get_color_frame()
        if color_frame:
            frame_count += 1
            print(f"✅ Got color frame! (frame {frame_count})")
            
            # Get data and display
            data = color_frame.get_data()
            if data is not None:
                img = np.asarray(data)
                if len(img.shape) == 3:
                    # If it's RGB, convert to BGR for OpenCV
                    if img.shape[2] == 3:
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    elif img.shape[2] == 4:
                        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                
                cv2.imshow("Orbbec Camera - Color", img)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:
                    break
        else:
            print("No color frame in this set")
    else:
        print("No frames received")

pipeline.stop()
cv2.destroyAllWindows()
print("Done!")