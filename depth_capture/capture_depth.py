import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

SAVE_DIR = "./depth_capture/saved_depth_maps"
os.makedirs(SAVE_DIR, exist_ok=True)

# ---------- RealSense 설정 ----------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()

# 프리셋/옵션(가능한 것만 시도)
try: depth_sensor.set_option(rs.option.visual_preset, 5)           # High Density
except: pass
for opt, val in [(rs.option.emitter_enabled,1.0),(rs.option.laser_power,180.0)]:
    try: depth_sensor.set_option(opt, float(val))
    except: pass
try:
    depth_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
    depth_sensor.set_option(rs.option.exposure, 10000.0)
    depth_sensor.set_option(rs.option.gain, 20.0)
except: pass

depth_scale = depth_sensor.get_depth_scale()

# ---------- 필터 ----------
decimation = rs.decimation_filter(); decimation.set_option(rs.option.filter_magnitude, 2)
spatial    = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)
temporal   = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)
holefill   = rs.hole_filling_filter(2)

apply_filters = True

# ---------- ROI 인터랙션 ----------
roi = None  # (x0, y0, size)
dragging = False
p0 = None

WIN = "Depth Preview (draw ROI: drag; X:clear; R:filter; SPACE:save)"
cv2.namedWindow(WIN)

def enforce_square(p0, p1):
    (x0, y0), (x1, y1) = p0, p1
    x0, x1 = sorted([x0, x1])
    y0, y1 = sorted([y0, y1])
    w = x1 - x0
    h = y1 - y0
    side = min(w, h)
    return (x0, y0, side)

def on_mouse(event, x, y, flags, param):
    global dragging, p0, roi
    if event == cv2.EVENT_LBUTTONDOWN:
        dragging = True
        p0 = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        x0, y0, side = enforce_square(p0, (x, y))
        roi = (x0, y0, side)
    elif event == cv2.EVENT_LBUTTONUP and dragging:
        dragging = False
        x0, y0, side = enforce_square(p0, (x, y))
        roi = (x0, y0, side)

cv2.setMouseCallback(WIN, on_mouse)

print("SPACE: save, R: toggle filter, X: clear ROI, ESC: quit")

def overlay_roi(img, roi):
    if roi is None: return img
    x0, y0, s = roi
    x1, y1 = x0 + s, y0 + s
    o = img.copy()
    cv2.rectangle(o, (x0, y0), (x1, y1), (0,255,0), 2)
    return o

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue

        show_frame = depth_frame
        if apply_filters:
            show_frame = decimation.process(show_frame)
            show_frame = spatial.process(show_frame)
            show_frame = temporal.process(show_frame)
            show_frame = holefill.process(show_frame)

        depth_image = np.asanyarray(show_frame.get_data())  # uint16

        vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        vis = overlay_roi(vis, roi)
        cv2.imshow(WIN, vis)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            break
        elif key == ord('r') or key == ord('R'):
            apply_filters = not apply_filters
            cv2.setWindowTitle(WIN, f"{WIN} [{'filtered' if apply_filters else 'raw'}]")
        elif key == ord('x') or key == ord('X'):
            roi = None
        elif key == 32:  # SPACE -> 저장
            ts = datetime.now().strftime("%Y_%m_%d_%H%M%S")
            base = os.path.join(SAVE_DIR, f"depth_{ts}{'_filtered' if apply_filters else '_raw'}")

            # --- ROI 크롭 ---
            H, W = depth_image.shape
            cropped = depth_image
            if roi is not None:
                x0, y0, s = roi
                x0 = max(0, min(W-1, x0))
                y0 = max(0, min(H-1, y0))
                s  = max(1, min(min(W-x0, H-y0), s))
                cropped = depth_image[y0:y0+s, x0:x0+s]

            # 저장
            np.save(base + ".npy", cropped)
            cv2.imwrite(base + "_raw16.png", cropped.astype(np.uint16))
            vis_crop = cv2.applyColorMap(
                cv2.convertScaleAbs(cropped, alpha=0.03), cv2.COLORMAP_JET
            )
            cv2.imwrite(base + "_vis.png", vis_crop)

            # 메타
            with open(base + "_meta.txt", "w", encoding="utf-8") as f:
                f.write(f"depth_scale(meters_per_unit)={depth_scale}\n")
                f.write(f"filters={'on' if apply_filters else 'off'}\n")
                f.write("stream=depth 1280x720@30, format=Z16\n")
                f.write(f"orig_size={H}x{W}\n")
                if roi is not None:
                    f.write(f"roi=x0:{x0},y0:{y0},size:{s}\n")
                else:
                    f.write("roi=None\n")
            print(f"✅ Saved: {base}.*  (roi={roi})")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
