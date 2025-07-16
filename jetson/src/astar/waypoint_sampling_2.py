import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between
import cv2

def is_clear_path(mask, p1, p2, kernel_size=3, max_violation_ratio=0.05):
    line_img = np.zeros_like(mask, dtype=np.uint8)
    cv2.line(line_img, (p1[1], p1[0]), (p2[1], p2[0]), 1, 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    corridor = cv2.dilate(line_img, kernel)
    corridor_mask = corridor.astype(bool)
    total_pixels = np.count_nonzero(corridor_mask)

    if total_pixels == 0:
        return False  # fallback: treat as blocked

    unsafe_pixels = np.count_nonzero(mask[corridor_mask] == 0)
    return unsafe_pixels / total_pixels <= max_violation_ratio

def sample_waypoints(path, obstacle_mask, angle_thresh=120, buffer=3, max_lookahead=100, step=10):
    from scipy.ndimage import distance_transform_edt
    free = obstacle_mask > 0
    dist_map = distance_transform_edt(free)
    height, width = dist_map.shape

    def too_close(pt):
        x, y = int(pt[0]), int(pt[1])
        if not (0 <= y < height and 0 <= x < width):  # y is row, x is column
            return True
        return dist_map[y, x] < buffer

    simplified = [path[0]]
    i = 0
    N = len(path)

    while i < N - 1:
        best = i + 1
        max_j = min(i + max_lookahead, N - 1)  # ← this is needed

        for j in range(i + step, max_j + 1, step):
            mid_idxs = [i + (j - i) // 3, i + 2 * (j - i) // 3]
            disqualified = False
            for k in mid_idxs:
                a = path[i]
                b = path[k]
                c = path[j]
                angle = angle_between(a, b, c)
                close = too_close(b)

                if close:
                    x, y = int(b[0]), int(b[1])
                    if 0 <= y < dist_map.shape[0] and 0 <= x < dist_map.shape[1]:
                        print(f"REJECTED: too_close @ {k} (dist_map={dist_map[y, x]:.2f})")
                    else:
                        print(f"REJECTED: too_close @ {k} (out of bounds x={x}, y={y})")
                    disqualified = True
                    break
                if angle < angle_thresh:
                    print(f"REJECTED: angle {angle:.1f}° < {angle_thresh} @ {k}")
                    disqualified = True
                    break

            if disqualified:
                continue

            if is_clear_path(obstacle_mask, path[i], path[j]):
                print(f"ACCEPTED: {i} → {j} via mid {mid_idxs}")
                best = j
            else:
                print(f"REJECTED: LOS failed from {i} → {j}")

        simplified.append(path[best])
        i = best


        simplified.append(path[best])
        i = best

    return simplified


