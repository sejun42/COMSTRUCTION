import numpy as np, cv2, sys, os, re, glob, time
from pathlib import Path

# ===== 기본 설정 (원하면 바꿔도 됨) =====
# 기본 탐색 경로: 스크립트 기준 ../depth_capture/saved_depth_maps
# 출력 경로:      스크립트 기준 ../terrain/heightmaps
DEFAULT_SIZE   = 513         # Unity Terrain heightmap resolution (2^n+1: 513/1025/2049...)
DEFAULT_INVERT = True        # True면 깊이가 클수록 더 높은 지형(필요에 따라 바꿔보세요)


def find_latest_pair(search_dir: Path):
    """ *_filtered.npy 우선, 없으면 *_raw.npy / 메타는
        1) depth파일명에서 .npy→_meta.txt
        2) '_filtered'를 제거한 뒤 _meta.txt
       두 패턴을 모두 시도해서 찾음
    """
    cands = sorted(
        list(search_dir.glob("*_filtered.npy")) + list(search_dir.glob("*.npy")),
        key=lambda p: p.stat().st_mtime
    )
    for depth_npy in reversed(cands):
        # 1) 같은 스템 + _meta.txt (기존 로직)
        meta1 = depth_npy.with_name(depth_npy.name.replace(".npy", "_meta.txt"))
        # 2) _filtered / _raw 태그 제거 후 _meta.txt
        stem = depth_npy.stem
        stem2 = stem.replace("_filtered","").replace("_raw","")
        meta2 = depth_npy.with_name(stem2 + "_meta.txt")
        if meta1.exists():
            return depth_npy, meta1
        if meta2.exists():
            return depth_npy, meta2
    return None, None

def load_depth_and_scale(depth_npy: Path, meta_txt: Path):
    meta = {}
    for line in open(meta_txt, encoding="utf-8"):
        m = re.match(r"([^=]+)=(.+)", line.strip())
        if m: meta[m.group(1)] = m.group(2)
    scale = float(meta.get("depth_scale(meters_per_unit)", "0.001"))  # fallback 1mm
    D = np.load(depth_npy).astype(np.float32)  # z16 depth units
    Z = D * scale                              # meters
    Z[D == 0] = np.nan
    return Z

def fill_holes_nearest(Z: np.ndarray):
    if not np.isnan(Z).any():
        return Z
    Zf = Z.copy()
    mmin, mmax = np.nanmin(Zf), np.nanmax(Zf)
    norm = (Zf - mmin) / (mmax - mmin + 1e-9)
    norm = np.nan_to_num(norm, nan=0.0).astype(np.float32)
    hole = np.isnan(Zf).astype(np.uint8)
    norm8 = (norm * 255).astype(np.uint8)
    filled8 = cv2.inpaint(norm8, hole, 3, cv2.INPAINT_NS)
    Zf = filled8.astype(np.float32) / 255.0 * (mmax - mmin) + mmin
    return Zf

def to_r16(H: np.ndarray, size: int, out_path: Path):
    H_resized = cv2.resize(H, (size, size), interpolation=cv2.INTER_LINEAR)
    hmin, hmax = np.nanmin(H_resized), np.nanmax(H_resized)
    Hn = (H_resized - hmin) / (hmax - hmin + 1e-9)
    H16 = np.clip(Hn * 65535.0, 0, 65535).astype(np.uint16)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(H16.tobytes(order="C"))
    cv2.imwrite(str(out_path.with_suffix(".png")), (Hn * 255).astype(np.uint8))
    return out_path

def main():
    # ---- 경로 자동 추정 ----
    script_dir = Path(__file__).resolve().parent
    default_in_dir  = (script_dir / ".." / "depth_capture" / "saved_depth_maps").resolve()
    default_out_dir = (script_dir / ".." / "terrain" / "heightmaps").resolve()

    # 인자 파싱 (있으면 우선)
    args = sys.argv[1:]
    size = DEFAULT_SIZE
    invert = DEFAULT_INVERT
    depth_npy = meta_txt = out_r16 = None

    for a in list(args):
        if a.startswith("--size="):
            size = int(a.split("=", 1)[1]); args.remove(a)
        elif a == "--invert":
            invert = True; args.remove(a)
        elif a == "--no-invert":
            invert = False; args.remove(a)

    if len(args) >= 3:
        depth_npy, meta_txt, out_r16 = map(Path, args[:3])
    else:
        # 오토모드: 기본 폴더에서 최신 파일 자동 탐색
        search_dir = default_in_dir if default_in_dir.exists() else script_dir
        dn, mt = find_latest_pair(search_dir)
        if dn is None:
            print("❌ 저장된 depth/메타 쌍을 찾지 못했습니다.")
            print(f"- 찾은 경로: {search_dir}")
            print("  예) *_filtered.npy 와 같은 이름의 *_meta.txt 가 있어야 합니다.")
            safe_exit()
            return
        depth_npy, meta_txt = dn, mt
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_r16 = default_out_dir / f"hm_{stamp}.r16"

    # ---- 변환 처리 ----
    print(f"▶ 입력 depth: {depth_npy}")
    print(f"▶ 입력 meta : {meta_txt}")
    print(f"▶ 출력 RAW  : {out_r16} (size={size}, invert={invert})")

    Z = load_depth_and_scale(depth_npy, meta_txt)  # meters, NaN set for invalid

    # 0) 유효 마스크
    valid = ~np.isnan(Z)

    # 1) 가장자리 수축(erosion)으로 경계 노이즈 제거
    kernel = np.ones((3,3), np.uint8)
    valid_eroded = cv2.erode(valid.astype(np.uint8), kernel, iterations=1).astype(bool)

    # 2) 자동 크롭(유효 픽셀의 바운딩 박스 + 여유 5px)
    ys, xs = np.where(valid_eroded)
    if len(xs) > 0:
        y0, y1 = max(0, ys.min()-5), min(Z.shape[0], ys.max()+6)
        x0, x1 = max(0, xs.min()-5), min(Z.shape[1], xs.max()+6)
        Z = Z[y0:y1, x0:x1]
        valid_eroded = valid_eroded[y0:y1, x0:x1]

    # 3) 결손 채움(최근접 스타일: inpaint 전처리 + 작은 median)
    Zf = Z.copy()
    if np.isnan(Zf).any():
        mmin = np.nanmin(Zf); mmax = np.nanmax(Zf)
        # 정규화 후 결손만 inpaint로 메움
        norm = (Zf - mmin) / (mmax - mmin + 1e-9)
        hole = np.isnan(norm).astype(np.uint8)
        norm8 = np.nan_to_num(norm, nan=0.0)
        norm8 = (norm8*255).astype(np.uint8)
        filled8 = cv2.inpaint(norm8, hole, 3, cv2.INPAINT_NS)
        Zf = filled8.astype(np.float32)/255.0*(mmax-mmin)+mmin
        # 작은 중값 필터로 스파이크 제거
        Zf = cv2.medianBlur(Zf, 3)

    # 4) 퍼센타일 클리핑으로 정규화 범위 안정화
    p_lo, p_hi = np.nanpercentile(Zf, [2, 98])
    Zc = np.clip(Zf, p_lo, p_hi)

    # 5) 높이 방향(멀수록 높게면 invert=True)
    H = Zc if invert else -Zc

    # 6) 리사이즈 → 16bit RAW 저장 (기존 로직과 동일)
    H_resized = cv2.resize(H, (size, size), interpolation=cv2.INTER_LINEAR)
    hmin, hmax = np.nanmin(H_resized), np.nanmax(H_resized)
    Hn = (H_resized - hmin) / (hmax - hmin + 1e-9)
    H16 = np.clip(Hn*65535.0, 0, 65535).astype(np.uint16)
    with open(out_r16, "wb") as f: f.write(H16.tobytes(order="C"))
    cv2.imwrite(str(Path(out_r16).with_suffix(".png")), (Hn*255).astype(np.uint8))

    out_path = to_r16(H, size, out_r16)
    print(f"✅ saved: {out_path}")
    print(f"🖼  preview: {out_path.with_suffix('.png')}")

def safe_exit():
    # 더블클릭 실행 시 창이 바로 닫히지 않도록 대기
    if not sys.stdin.isatty():
        try:
            input("엔터를 누르면 창이 닫힙니다...")
        except Exception:
            pass

if __name__ == "__main__":
    try:
        main()
    finally:
        safe_exit()
