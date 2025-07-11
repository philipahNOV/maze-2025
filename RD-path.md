# A* Pathfinding

This part explains the architecture and implementation of a complete A* pathfinding system built for navigating the maze using binary obstacle maps. The system includes repulsion-aware path planning, mask preprocessing, waypoint smoothing, path memory caching, and several geometric enhancements. It is modular and can generalize for any type of maze with darker walls and a light background.

---

## 1. A* Algorithm with Repulsion Field

The core of the system is a modified A* algorithm (`astar.py`) that adds a **repulsion term** to avoid planning paths too close to obstacles. The algorithm uses **Manhattan distance** as its heuristic and introduces a **cost penalty** that increases near obstacles using a distance transform.

### Heuristic

The Manhattan distance works well for grid-based maps:

```python
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
```

### Repulsion Map

Repulsion cost is calculated by a distance transform on the binary obstacle map:

```python
def compute_repulsion_cost(array, min_safe_dist=14):
    dist_transform = cv2.distanceTransform((array * 255).astype(np.uint8), cv2.DIST_L2, 3)
    repulsion = (1.0 - (dist_transform / max_dist)) ** 3.0
    return repulsion, mask_safe
```

This produces a smooth cost field where grid points near obstacles are heavily penalized.

### Path Search

The `astar()` function integrates the repulsion field into the node cost:

```python
cost = 1.0 + repulsion_weight * repulsion_map[neighbor[0], neighbor[1]]
tentative_g = gscore[current] + cost
```

This guides the A* search away from narrow gaps and obstacle boundaries.

---

## 2. Downscaled Pathfinding

The `astar_downscaled()` function provides a mechanism to run A* on a lower-resolution version of the map for performance or smoothing purposes:

```python
small_array = cv2.resize(array, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
path_small = astar(small_array, start_small, goal_small, repulsion_weight)
```

Once a path is found, it is scaled back up to the original resolution.

---

## 3. Binary Mask Preprocessing

Binary masks are generated from RGB camera input and refined through a series of transformations in `mask_utils.py`. The goal is to extract a reliable navigable area that excludes noise, green-field artifacts, and subtle edges.

### CLAHE and Thresholding

The grayscale image is equalized with CLAHE and then binarized using Otsu’s threshold:

```python
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
enhanced = apply_clahe(gray)
_, base_mask = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
```

### Edge Removal and Morphology

Edge noise is removed and the mask is cleaned:

```python
edges = cv2.Canny(enhanced, 100, 200)
edges_inv = cv2.bitwise_not(cv2.dilate(edges, kernel))
final_mask = cv2.bitwise_and(cleaned, edges_inv)
```

### Green Area Suppression

Green pixels from the ball are removed:

```python
hsv = cv2.cvtColor(gray_to_bgr, cv2.COLOR_BGR2HSV)
green_mask = cv2.inRange(hsv, lower_green, upper_green)
final_mask = cv2.bitwise_or(final_mask, green_mask)
```

---

## 4. Waypoint Sampling

The raw A* path is often jagged or too dense for real-time movement. The `waypoint_sampling.py` module extracts smooth and valid waypoints with spacing and angular constraints.

### Spacing and Angles

Waypoints are selected based on distance and turning angles:

```python
angle = angle_between(path[last_wp_idx], path[i], path[i + 1])
if angle < angle_threshold and accumulated >= spacing / 2:
    if is_clear_path(mask, path[last_wp_idx], path[i]):
        waypoints.append(path[i])
```

### Visibility Checks

Every candidate waypoint is validated against a morphological corridor in the mask:

```python
def is_clear_path(mask, p1, p2, kernel_size=6):
    cv2.line(line_img, p1, p2, 1, 1)
    corridor = cv2.dilate(line_img, kernel)
    return np.all(mask[corridor.astype(bool)] > 0)
```

---

## 5. Path Drawing

The `draw_path.py` module overlays paths on images for debugging or visualization:

```python
def draw_path(image, waypoints, start, goal):
    for x, y in waypoints:
        cv2.circle(out, (x, y), 8, (0, 0, 255), -1)
```

The output supports grayscale or color images, and annotates start/goal locations.

---

## 6. Path Memory Caching

To avoid recomputation, `path_memory.py` stores previously computed paths using a fuzzy match based on Euclidean distance.

```python
def get_cached_path(self, start, goal):
    if _within_tolerance(start, cached_start) and _within_tolerance(goal, cached_goal):
        return cached_path
```

Paths are serialized to JSON and persisted across runs:

```python
with open(self.cache_file, "w") as f:
    json.dump(self.paths, f)
```

Up to `max_paths` are retained and replaced in FIFO order.

---

## 7. Nearest Walkable Point

The function `find_nearest_walkable()` makes sure that the start and goal positions fall inside valid areas:

```python
if mask[y, x] != 0:
    return point
```

If not, it searches in a radius using a distance transform to locate the nearest walkable pixel.

---

## 8. Known Issues and Tuning

### repulsion_weight

Controls how strongly the path avoids obstacles. Too low and the path hugs walls; too high and paths become over-conservative.

```python
astar(..., repulsion_weight=5.0)
```

Try values between 2.0 and 6.0 for balance.

### min_safe_dist in repulsion

Defines how far from an obstacle a point must be to be considered "safe." Experience has shown that somewhere between 10 and 15 is the sweet spot. Larger values create wider margins:

```python
compute_repulsion_cost(..., min_safe_dist=14)
```

Increase to reduce path tightness.

### scale in `astar_downscaled()`

In this implementation it's not currently used (scale=1.0), but here the scale factor can be adjusted to control how much the input map is shrunk. Lower values increase smoothness, loading time and performance but reduce precision and can make A* fail in downscaled space.

```python
astar_downscaled(..., scale=1.0)
```

### waypoint_spacing and angle_threshold

Tune these in `sample_waypoints()` to balance turning smoothness with motion efficiency:

```python
waypoint_spacing=160, angle_threshold=135 # default values
```

Higher spacing means fewer waypoints. Lower angle thresholds increase turning flexibility.

### cache tolerance

Controls how closely a cached path must match a new request to be reused:

```python
PathMemory(tolerance=10)
```

Increase to enable more cache hits but risk path mismatch.