import pyrealsense2 as rs
import numpy as np
import cv2

# 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()

# D455 스트림 설정 (640x480, 30fps)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 스트리밍 시작
pipeline.start(config)

# 깊이 스케일 (depth units → meters)
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"[INFO] Depth Scale (m per unit): {depth_scale} → {depth_scale * 1000:.3f} mm per unit")

# 깊이 범위 설정 (컬러맵용)
min_depth = 100   # 30cm (빨강)
max_depth = 400  # 100cm (보라/파랑)

# 마우스 콜백: 해당 위치 거리(mm) 출력
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        depth_frame = param
        if depth_frame and depth_frame.get_distance(x, y) > 0:
            dist = depth_frame.get_distance(x, y) * 1000  # mm
            print(f"Cursor at ({x},{y}) → 거리: {dist:.1f} mm")

cv2.namedWindow("Depth Visualization")
cv2.setMouseCallback("Depth Visualization", mouse_callback, None)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # depth image: uint16 (단위: mm)
        depth_image = np.asanyarray(depth_frame.get_data())

        # 클리핑: min~max 범위로 제한
        clipped_depth = np.clip(depth_image, min_depth, max_depth)

        # 정규화: 0~255 (uint8)
        normalized_depth = ((clipped_depth - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

        # 컬러맵 적용 (빨주노초파남보)
        display_image = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

        # 마우스 이벤트 전달용
        cv2.setMouseCallback("Depth Visualization", mouse_callback, depth_frame)

        # 이미지 표시
        cv2.imshow("Depth Visualization", display_image)

        key = cv2.waitKey(1)
        if key == ord('s'):
            np.save("depth_raw_mm.npy", depth_image)
            print("[✔] depth_raw_mm.npy 저장 완료 (단위: mm)")
        elif key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
