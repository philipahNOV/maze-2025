import os
import cv2
import pyzed.sl as sl

def init_zed_camera():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 1280×720 @ 30 FPS
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED camera failed to open.")
        exit(1)

    return zed

def grab_zed_frame(zed):
    mat = sl.Mat()
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # retrieve 3-channel BGR directly
        zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
        bgr = mat.get_data()            # shape (720,1280,3), BGR
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return rgb, bgr
    else:
        return None, None

def main():
    # parameters
    save_dir = "captures"
    os.makedirs(save_dir, exist_ok=True)
    img_count = 0

    # init
    zed = init_zed_camera()
    cv2.namedWindow("ZED RGB", cv2.WINDOW_NORMAL)

    try:
        while True:
            frame_rgb, frame_bgr = grab_zed_frame(zed)
            if frame_rgb is None:
                print("Error: Failed to grab frame.")
                break

            # display the RGB view
            cv2.imshow("ZED RGB", frame_rgb)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # SPACE pressed
                filename = os.path.join(save_dir, f"image_{img_count:03d}.jpg")
                # save the *BGR* data so color is correct in the JPG
                cv2.imwrite(filename, frame_bgr)
                print(f"Saved {filename}")
                img_count += 1

            elif key == ord('q'):  # 'q' to quit
                break

    finally:
        zed.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
