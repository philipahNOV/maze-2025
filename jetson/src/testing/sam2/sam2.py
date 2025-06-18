import cv2
import numpy as np
import time
from segment_anything import SamPredictor, sam_model_registry

def load_sam2_model(model_type="vit_h", checkpoint_path="sam_vit_h_4b8939.pth"):
    """
    Loads the SAM2 model for segmentation.
    """
    sam = sam_model_registry[model_type](checkpoint=checkpoint_path)
    predictor = SamPredictor(sam)
    return predictor

def track_ball_sam2(video_name, predictor, prev_mask=None):
    """
    Tracks the ball in real time using SAM2 segmentation.
    """
    vid = cv2.VideoCapture(video_name)
    if not vid.isOpened():
        print("Error: Canot open video file.")
        exit()

    frame_num = 0
    prev_center = None
    while True:
        ret, frame = vid.read()
        if not ret:
            print("End of video or error reading frame.")
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        predictor.set_image(rgb_frame)

        if prev_mask is not None:
            masks, scores, logits = predictor.predict(
                mask_input=prev_mask,
                multimask_output=False
            )
        else:
            # For the first frame, use a point prompt at the image center or a rough ROI
            h, w, _ = frame.shape
            input_point = np.array([[w // 2, h // 2]])
            input_label = np.array([1])
            masks, scores, logits = predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                multimask_output=False
            )

        mask = masks[0].astype(np.uint8) * 255
        prev_mask = logits[0]  # Use logits as mask input for next frame

        # Find the largest contour in the mask (assume it's the ball)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = prev_center
        radius = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest)
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 0), 4)
            prev_center = center

        cv2.imshow('SAM2 Ball Tracking', frame)
        frame_num += 1
        print(f"Frame {frame_num}, Center: {center}, Radius: {radius}")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    vid.release()
    cv2.destroyAllWindows()

def main():
    predictor = load_sam2_model(model_type="vit_h", checkpoint_path="sam_vit_h_4b8939.pth")
    track_ball_sam2("output.mp4", predictor)

if __name__ == "__main__":
    main()