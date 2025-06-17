import cv2
import pyzed.sl as sl

def test():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = True

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Camera Open Error")
        return False
    
    print("Capture started")
    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()

    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        cv2.imshow("ZED Image", img)
        cv2.imwrite("zed_test_image.jpg", img)  

        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Image captured and displayed successfully")
    else:
        print("Failed to grab image from camera")

    zed.close()

if __name__ == "__main__":
    try:
        test()
    except Exception as e:
        print(f"An error occurred: {e}")
