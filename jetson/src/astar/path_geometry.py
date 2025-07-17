def clear_path(p0, p1, mask):
    y0, x0 = p0; y1, x1 = p1
    dy, dx = abs(y1 - y0), abs(x1 - x0)
    sy, sx = (1 if y0 < y1 else -1), (1 if x0 < x1 else -1)
    err = dx - dy; y, x = y0, x0
    while True:
        if not mask[y][x]:
            return False
        if (y, x) == (y1, x1):
            return True
        e2 = err*2
        if e2 > -dy:
            err -= dy; x += sx
        if e2 <  dx:
            err += dx; y += sy