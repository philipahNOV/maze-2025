import cv2
import os

def main():
    cap = cv2.VideoCapture(0)  # Change to 1 or 2 if needed
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    save_dir = "img"
    os.makedirs(save_dir, exist_ok=True)

    img_count = 0
    print("Press SPACE to take a picture.")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break

        cv2.imshow("Camera Feed", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # SPACE pressed
            filename = os.path.join(save_dir, f"image_{img_count:03d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            img_count += 1

        elif key == ord('q'):  # Quit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
