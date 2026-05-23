import argparse
from pathlib import Path

import cv2


DEFAULT_CALIB_DIR = Path(__file__).resolve().parent / "calibration"
WINDOW_NAME = "LaserAimer boresight uv calibration"
RAW_CAMERA_MATRIX = (
    (2847.305415, 0.0, 658.403337),
    (0.0, 2846.695845, 532.415900),
    (0.0, 0.0, 1.0),
)
DIST_COEFFS = (-0.065828, 0.983370, 0.004467, 0.001856, 0.0)
RECTIFIED_CAMERA_MATRIX = (
    (2854.342510, 0.0, 659.172967),
    (0.0, 2851.239471, 534.105012),
    (0.0, 0.0, 1.0),
)


class MouseState:
    def __init__(self) -> None:
        self.x = -1
        self.y = -1
        self.inside = False
        self.clicked = None


def collect_images(calib_dir: Path, limit: int) -> list[Path]:
    images = sorted(calib_dir.glob("*.png"), key=lambda p: p.name.lower())
    if limit > 0:
        images = images[:limit]
    return images


def undistort_image(image):
    import numpy as np

    camera_matrix = np.array(RAW_CAMERA_MATRIX, dtype=np.float64)
    dist_coeffs = np.array(DIST_COEFFS, dtype=np.float64)
    new_camera_matrix = np.array(RECTIFIED_CAMERA_MATRIX, dtype=np.float64)
    return cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)


def on_mouse(event, x, y, flags, userdata) -> None:
    state: MouseState = userdata
    if event == cv2.EVENT_MOUSEMOVE:
        state.x = x
        state.y = y
        state.inside = True
        print(f"\ru={x:4d}, v={y:4d}", end="", flush=True)
    elif event == cv2.EVENT_LBUTTONDOWN:
        state.clicked = (x, y)
        print(f"\nselected: u={x}, v={y}")


def draw_overlay(image, index: int, total: int, path: Path, state: MouseState):
    view = image.copy()
    h, w = view.shape[:2]
    center = (w // 2, h // 2)

    cv2.line(view, (center[0], 0), (center[0], h - 1), (0, 255, 0), 1, cv2.LINE_AA)
    cv2.line(view, (0, center[1]), (w - 1, center[1]), (0, 255, 0), 1, cv2.LINE_AA)
    cv2.drawMarker(view, center, (0, 255, 0), cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA)

    cv2.putText(
        view,
        f"image {index + 1}/{total}: {path.name}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        view,
        f"center: u={center[0]}, v={center[1]}",
        (12, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    if 0 <= state.x < w and 0 <= state.y < h:
        p = (state.x, state.y)
        cv2.line(view, (p[0], 0), (p[0], h - 1), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(view, (0, p[1]), (w - 1, p[1]), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(
            view,
            f"cursor: u={p[0]}, v={p[1]}",
            (12, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    if state.clicked:
        cv2.putText(
            view,
            f"selected: u={state.clicked[0]}, v={state.clicked[1]}",
            (12, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 128, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.putText(
        view,
        "keys: Space/n next | p prev | c clear | q/Esc quit | left click select",
        (12, h - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return view


def main() -> int:
    parser = argparse.ArgumentParser(description="Read calibration PNGs and inspect boresight u/v.")
    parser.add_argument("--dir", default=str(DEFAULT_CALIB_DIR), help="Calibration image directory.")
    parser.add_argument("--limit", type=int, default=5, help="How many PNG images to read, 0 means all.")
    parser.add_argument("--raw", action="store_true", help="Show raw images without undistortion.")
    args = parser.parse_args()

    calib_dir = Path(args.dir).expanduser().resolve()
    images = collect_images(calib_dir, args.limit)
    if not images:
        print(f"No png images found in: {calib_dir}")
        return 1

    frames = []
    for path in images:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"Skip unreadable image: {path}")
            continue
        if not args.raw:
            image = undistort_image(image)
        frames.append((path, image))

    if not frames:
        print("No readable images.")
        return 1

    mode = "raw" if args.raw else "undistorted"
    print(f"Loaded {len(frames)} {mode} image(s) from {calib_dir}")
    print("Move mouse over the image to print u/v. Left click to keep a selected coordinate.")

    state = MouseState()
    index = 0
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse, state)

    while True:
        path, image = frames[index]
        view = draw_overlay(image, index, len(frames), path, state)
        cv2.imshow(WINDOW_NAME, view)
        key = cv2.waitKey(16) & 0xFF

        if key in (27, ord("q")):
            break
        if key in (ord(" "), ord("n"), ord("d")):
            index = (index + 1) % len(frames)
            state.clicked = None
        elif key in (ord("p"), ord("a")):
            index = (index - 1) % len(frames)
            state.clicked = None
        elif key == ord("c"):
            state.clicked = None

    print()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
