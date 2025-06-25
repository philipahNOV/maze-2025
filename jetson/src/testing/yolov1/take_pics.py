import os
import cv2
import pyzed.sl as sl

def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # or other resolution
    init_params.camera_fps = 30

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error: {status}")
        exit(1)

    save_dir = "img2"
    os.makedirs(save_dir, exist_ok=True)

    img_count = 0
    print("Press SPACE to take a picture.")
    print("Press 'q' to quit.")

    image = sl.Mat()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            # Convert RGBA to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            cv2.imshow("ZED Camera Feed", frame_bgr)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # SPACE pressed
                filename = os.path.join(save_dir, f"image_{img_count:03d}.jpg")
                cv2.imwrite(filename, frame_bgr)
                print(f"Saved {filename}")
                img_count += 1

            elif key == ord('q'):  # Quit
                break
        else:
            print("Error: Failed to grab frame.")
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
