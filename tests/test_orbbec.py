import cv2
import numpy as np
import pyorbbecsdk
from pyorbbecsdk import Pipeline, FrameSet, OBSensorType, OBFormat

def main():
    # Initialize the camera pipeline
    pipeline = Pipeline()
    try:
        pipeline.start(None)
        print("Camera stream started successfully! Press 'q' to exit.")
    except pyorbbecsdk.OBError as e:
        print(f"Failed to start pipeline: {e}")
        print("Please check your USB 3.0 connection and camera drivers.")
        return

    while True:
        try:
            # Wait for a synchronized frame set (timeout 100ms)
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue

            # 1. Process Color Frame
            color_frame = frames.get_color_frame()
            if color_frame is not None:
                color_data = color_frame.get_data()
                # Orbbec SDK generally outputs RGB or BGR arrays
                color_image = np.frombuffer(color_data, dtype=np.uint8)
                color_image = color_image.reshape((color_frame.get_height(), color_frame.get_width(), 3))
                # Convert RGB to OpenCV default BGR if colors look swapped
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                cv2.imshow("Orbbec - Color Stream", color_image)

            # 2. Process Depth Frame
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None:
                depth_data = depth_frame.get_data()
                depth_image = np.frombuffer(depth_data, dtype=np.uint16)
                depth_image = depth_image.reshape((depth_frame.get_height(), depth_frame.get_width()))
                
                # Normalize 16-bit depth values to 8-bit for visibility
                depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                cv2.imshow("Orbbec - Depth Stream", depth_colored)

        except Exception as e:
            print(f"Error reading frames: {e}")
            break

        # Break the loop when user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and release system memory
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Streams closed safely.")

if __name__ == "__main__":
    main()
