import os
import cv2
import pyzed.sl as sl

def init_zed_camera():
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD2K
    params.camera_fps = 15
    params.coordinate_units = sl.UNIT.MILLIMETER
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); exit(1)
    return zed

def grab_zed_frame(zed):
    mat = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None, None
    # 1) grab 3-channel BGR
    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
    bgr = mat.get_data()
    # 2) make an RGB copy if you need it elsewhere
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb, bgr

def main():
    zed = init_zed_camera()
    save_dir = "captures2"
    os.makedirs(save_dir, exist_ok=True)
    img_count = 26

    # create a small, resizable window
    win_name = "ZED View"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)  # half of 1280×720

    try: 
        while True:
            rgb, bgr = grab_zed_frame(zed)
            if bgr is None:
                print("Frame grab failed"); break

            # Display the BGR image directly:
            cv2.imshow(win_name, bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # SPACE: save BGR so colors stay correct
                fn = os.path.join(save_dir, f"img_{img_count+1}.png")
                cv2.imwrite(fn, bgr)
                print("Saved", fn)
                img_count += 1
            elif key == ord('q'):  # Quit
                break
    finally:            
        zed.close()
        cv2.destroyAllWindows()
        

if __name__ == "__main__":
    main()