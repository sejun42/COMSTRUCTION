import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# 저장 폴더 경로
SAVE_DIR = "./saved_depth_maps"
os.makedirs(SAVE_DIR, exist_ok=True)

# 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 시작
pipeline.start(config)
align = rs.align(rs.stream.color)

print("📸 SPACE 키를 눌러 Depth 저장 / ESC로 종료")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())

        # 컬러맵 시각화
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        cv2.imshow('Depth Preview', depth_colormap)

        key = cv2.waitKey(1)
        if key == 32:  # SPACE
            timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
            np.save(os.path.join(SAVE_DIR, f"depth_{timestamp}.npy"), depth_image)
            cv2.imwrite(os.path.join(SAVE_DIR, f"depth_{timestamp}.png"), depth_colormap)
            print(f"✅ 저장 완료: depth_{timestamp}.npy / .png")

        elif key == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
