"""
MaixCAM-Pro 统一视觉程序 - JSON 串口协议版

依据：视觉数据传输约定（精简版）V1.0
参考：classdesign_v6.py + green_v4.py

运行环境：MaixVision 连接 MaixCAM-Pro 后运行。

上电识别顺序：
1. 第一个连续稳定识别到的绿光锁定为被追踪目标；
2. 目标锁定后，第二个连续稳定识别到的绿光锁定为追踪点；
3. 两者锁定后按上一帧位置做最近邻匹配，保持身份不随亮度变化；
4. 两点重合后，重新分离时先离开重合中心的光点锁定为被追踪目标。
5. 两点共同运动且方向稳定时，运动方向前方的光点作为被追踪目标，后方光点作为追踪光。

输出：
1. VISION_RESULT：被追踪目标毫米坐标 + 4个目标顶点毫米坐标；
2. TRACK_RESULT：被追踪目标毫米坐标 + 追踪点毫米坐标。

为兼容现有 STM32 协议，字段名保持不变：red_spot 表示被追踪目标，
green_spot 表示追踪点。下位机仍按“green_spot 追 red_spot”控制。

每个 JSON 对象单独占一行，以 \r\n 结束。

程序流程：
摄像头取图 -> 识别A4框和绿光 -> 按出现顺序分配身份 -> 像素转毫米 -> 生成JSON
-> UART1自动发送 -> 屏幕显示和终端调试。

注意：UART为自动发送模式，不需要等待STM32发送启动命令。
"""

from maix import camera, display, app, pinmap, uart, image, time, err
import math

# 1. 用户配置区（现场主要修改这里）

IMAGE_W = 320
IMAGE_H = 240

# 当前任务："A4" 或 "SQUARE"。
# A4：视觉识别任意位置、任意旋转的A4黑胶带框。
# SQUARE：直接输出屏幕中心0.5m正方形的固定毫米顶点。
TARGET_MODE = "A4"

# 程序版本：启动日志第一行用于确认板端运行的是当前文件。
PROGRAM_VERSION = "v6.11-overlap-first-mover-2cm"

# 镜头径向畸变校正（官方 MaixPy Image API: lens_corr）。
# 四角单应只能保证4个角点准确，斜线/对角线中间点误差大时优先打开。
# strength 需在板端按实际画面微调：校正不足时改大，校正过头出现反向弯曲时改小。
# 当前镜头在 strength=1.8 时出现明显反向弯曲，先关闭校正并使用原始画面。
# 若以后重新启用，必须从较小 strength 开始并重新验证 A4 坐标精度。
ENABLE_LENS_CORR = False
LENS_CORR_STRENGTH = 1.8
LENS_CORR_ZOOM = 1.0
LENS_CORR_X = 0.0
LENS_CORR_Y = 0.0

# 两束光不再按强弱分配身份。两个阈值只用于覆盖普通绿色光斑和可能
# 过曝的高亮核心；find_blobs 会把它们作为同一类绿光候选统一处理。
GREEN_THRESHOLDS = [
    [76, 80, -128, -8, -128, 127],
    [85, 100, -128, 127, -128, 127],
]

# 激光落到A4黑色电工胶布边框上时，背景变暗，使用现场验证过的备用LAB阈值。
# 程序会先用上面的普通阈值检测整张纸，再只在A4黑边区域接收本阈值的候选点。
GREEN_BLACK_BORDER_THRESHOLDS = [
    [15, 100, -100, -4, -25, 100],
]

# 黑色胶布边框的有效带宽，单位为毫米。胶布较宽或A4识别有抖动时可适当调大。
BLACK_BORDER_BAND_MM = 18.0
BLACK_BORDER_OUTER_MARGIN_MM = 6.0
GREEN_CANDIDATE_MERGE_DISTANCE_PX = 12

# 原因2实验版：当同一个激光在黑框上产生反射或光晕时，可能被拆成两个候选。
# 距离主光斑太近、面积明显更小的候选会被当成反射/光晕丢弃。
GREEN_HALO_SUPPRESS_DISTANCE_MM = 18.0
GREEN_HALO_SUPPRESS_DISTANCE_PX = 20
GREEN_HALO_AREA_RATIO = 0.70

# 暗光下绿点偏小偏暗，降到 1 减少漏检；噪点增多时再调回。
BLOB_PIXELS_THRESHOLD = 1
# 第一束和第二束绿光都需连续多帧稳定出现后才锁定。
FIRST_GREEN_CONFIRM_FRAMES = 3
SECOND_GREEN_CONFIRM_FRAMES = 3
GREEN_STABLE_TOLERANCE_PX = 12
# 已锁定光点的小位移直接更新；较大跳变需重新连续确认。
GREEN_TRACK_TOLERANCE_PX = 50
GREEN_REACQUIRE_MAX_JUMP_PX = 90
# 激光点通常是小而亮的点，过滤面积过大的区域；按现场光斑大小调整。
GREEN_MAX_BBOX_AREA = 1000
# 已识别绿光周围半径 1 cm 内不再识别为另一束光。
GREEN_ROLE_MIN_SEPARATION_MM = 10.0
# 两种身份分配的位移分数接近时，保持上一帧身份，避免近距离/交叉时互换。
GREEN_ASSIGNMENT_AMBIGUITY_MARGIN_PX = 12
# 候选明显更接近另一角色时，拒绝这次交叉分配，保持原角色。
GREEN_ROLE_CROSSOVER_MARGIN_PX = 6
# find_rects 的 threshold 语义以官方文档为准；若检测不到A4框，先降低此值并在板端实测。
RECT_THRESHOLD = 80000
RECT_MIN_BOUNDING_AREA = 1500

# 连续多帧识别到相近A4矩形后才锁定，防止锁定反光或错误矩形。
RECT_CONFIRM_FRAMES = 5
RECT_STABLE_TOLERANCE_PX = 12

# A4四角按中心缩放。1.00走A4边缘；需要绿光轨迹更靠内时减小。
RECT_PATH_SCALE = 1.00

# 中间矩形上边整体下移比例：0.08表示上边比矩形高度低8%，长度与下边保持一致。
RECT_TOP_LOWER_RATIO = 0.00

# 中间矩形下边整体上移比例：0.08表示下边比矩形高度高8%（上移），长度与上边保持一致。
RECT_BOTTOM_RAISE_RATIO = 0.00

# 左右竖直边分别向黑胶带外侧补偿的距离，单位为毫米。
# A4坐标中左边为X正方向，右边为X负方向；只修正左右边，不改变上下边。
# 当前根据实测误差设置：左边总外扩约19 mm，右边总外扩约9 mm。
A4_LEFT_EDGE_OUTWARD_MM = 19.0
A4_RIGHT_EDGE_OUTWARD_MM = 9.0

# A4斜放时保留黑框相对相机/执行机构坐标系的旋转角。
# True：输出的毫米顶点和两束绿光坐标随黑框旋转；False：兼容旧版的水平矩形坐标。
PRESERVE_A4_ROTATION = True

# A4锁定后ROI在A4角点外扩的边距；边距越大白框越大，但搜索区域也更宽。
TRACKING_ROI_MARGIN = 25

# 两束绿光短暂重合时可能无法分离，允许保留上一位置若干帧。
BLOB_HOLD_FRAMES = 3

# 两束光接近/合并时进入“重合锁定”状态。重新分离后，先离开重合中心
# 的光斑直接判定为被追踪目标，避免两个角色在交叉处交换。
GREEN_OVERLAP_MAX_DISTANCE_PX = 12
# 重合后按标定毫米距离判定“先离开”的激光。达到1 cm才触发目标锁定。
GREEN_OVERLAP_FIRST_MOVE_THRESHOLD_MM = 20.0
# 两点都达到1 cm时，只有位移差达到该值才按先后判定；否则继续等待。
GREEN_OVERLAP_FIRST_MOVE_MARGIN_MM = 2.0

# 运动方向判别：当两点存在稳定的整体运动时，把运动方向上更靠前的
# 光点分配为被追踪目标，后面的光点分配为追踪光。位移不足或前后差太小
# 时不改身份，避免抖动导致交换。
GREEN_DIRECTION_SPEED_THRESHOLD_PX = 3
GREEN_DIRECTION_FRONT_MARGIN_PX = 4
GREEN_DIRECTION_CONFIRM_FRAMES = 2

# 测试用：本帧没识别到目标绿光时强制 valid=1 会让下位机在目标锁定前就启动轨迹，
# 表现为开机后直接往某个顶点跑。默认关闭；只在下位机解析调试时临时打开。
FORCE_VISION_VALID = False
TARGET_FALLBACK_MM = [0.0, 0.0]

# 坐标原点：True 时以第一束目标绿光第一次确认出现的位置作为 (0,0)，
# 两个光点和四个目标顶点都输出相对该原点的毫米坐标。
USE_FIRST_GREEN_AS_ORIGIN = True

# 每10帧打印一次调试信息。
DEBUG_PRINT_EVERY = 10

# True：程序启动后自动向STM32/串口助手发送，不需要接收启动命令。
# False：只运行视觉识别，不从UART引脚发送数据。
ENABLE_UART = True

# VISION_RESULT 保持原用途；TRACK_RESULT 用于第二束绿光追踪第一束绿光。
ENABLE_VISION_RESULT = True

# MaixCAM-Pro 使用UART1做自定义通信：A19发送、A18接收。
# 配套转接接口上的TX/RX通常属于UART0，不要与这里混接。
UART_DEVICE = "/dev/ttyS1"
UART_RX_PIN = "A18"
UART_TX_PIN = "A19"
UART_RX_FUNCTION = "UART1_RX"
UART_TX_FUNCTION = "UART1_TX"

# 串口协议参数：Word约定115200、8-N-1。
UART_BAUDRATE = 115200

# 固定25Hz发送，满足协议“建议30Hz、最低20Hz”，同时给JSON串口带宽留余量。
SEND_FREQUENCY_HZ = 25
SEND_INTERVAL_MS = int(1000 / SEND_FREQUENCY_HZ)
# UART 错误日志限流间隔，防止断线时刷屏拖慢主循环。
UART_ERROR_LOG_INTERVAL_MS = 1000

# 2. 像素坐标到毫米坐标的标定

# 联调阶段临时打开：A4锁定并建立标定后即可输出 valid=1。
CALIBRATION_READY = True

# SQUARE 模式仍使用固定的0.5m正方形标定。
# A4 模式不使用本组角点，标定只来自运行时识别到的黑框四角。
# 硬件搭好后，把摄像头画面中“0.5m正方形”的四个角填在这里，
# 顺序必须是：左上、右上、右下、左下。
CALIBRATION_PIXEL_CORNERS = [
    [60.0, 20.0],
    [260.0, 20.0],
    [260.0, 220.0],
    [60.0, 220.0],
]

# SQUARE 模式：坐标原点为0.5m正方形中心；输出X左正、Y上正。
CALIBRATION_MM_CORNERS = [
    [-250.0, 250.0],
    [250.0, 250.0],
    [250.0, -250.0],
    [-250.0, -250.0],
]

# A4 模式：A4纸已知尺寸 210 x 297 mm，检测到四角后直接建立单应标定，
# 坐标原点为A4纸中心，输出X左正、Y上正，无需手填像素角点。
# 横放/竖放对应不同的毫米角点，运行时按检测到的长边方向选择。
A4_MM_CORNERS_PORTRAIT = [
    [-105.0, 148.5],
    [105.0, 148.5],
    [105.0, -148.5],
    [-105.0, -148.5],
]
A4_MM_CORNERS_LANDSCAPE = [
    [-148.5, 105.0],
    [148.5, 105.0],
    [148.5, -105.0],
    [-148.5, -105.0],
]

# 3. 基础工具函数

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def point_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def order_corners_clockwise(corners):
    """固定为左上、右上、右下、左下（图像中为顺时针）。"""
    points = [[float(p[0]), float(p[1])] for p in corners]
    cx = sum(p[0] for p in points) / 4.0
    cy = sum(p[1] for p in points) / 4.0

    points.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    start = min(range(4), key=lambda i: points[i][0] + points[i][1])
    points = points[start:] + points[:start]

    return [[int(round(p[0])), int(round(p[1]))] for p in points]


def scale_corners(corners, scale):
    cx = sum(p[0] for p in corners) / 4.0
    cy = sum(p[1] for p in corners) / 4.0
    result = []

    for point in corners:
        result.append([
            int(round(cx + (point[0] - cx) * scale)),
            int(round(cy + (point[1] - cy) * scale)),
        ])

    return result


def lower_top_edge(corners, lower_ratio):
    """把上边两角沿矩形高度方向整体下移，长度不变；0.0不移动。"""
    if lower_ratio is None or lower_ratio <= 0.0:
        return [list(point) for point in corners]

    height_dx = corners[3][0] - corners[0][0]
    height_dy = corners[3][1] - corners[0][1]
    height = math.sqrt(height_dx * height_dx + height_dy * height_dy)
    if height < 1.0:
        return [list(point) for point in corners]

    unit_x = height_dx / height
    unit_y = height_dy / height
    offset = height * lower_ratio

    result = [list(point) for point in corners]
    for index in (0, 1):
        result[index][0] = int(round(corners[index][0] + unit_x * offset))
        result[index][1] = int(round(corners[index][1] + unit_y * offset))
    return result


def raise_bottom_edge(corners, raise_ratio):
    """把下边两角沿矩形高度方向整体上移，长度不变；0.0不移动。"""
    if raise_ratio is None or raise_ratio <= 0.0:
        return [list(point) for point in corners]

    height_dx = corners[3][0] - corners[0][0]
    height_dy = corners[3][1] - corners[0][1]
    height = math.sqrt(height_dx * height_dx + height_dy * height_dy)
    if height < 1.0:
        return [list(point) for point in corners]

    unit_x = height_dx / height
    unit_y = height_dy / height
    offset = height * raise_ratio

    result = [list(point) for point in corners]
    for index in (2, 3):
        result[index][0] = int(round(corners[index][0] - unit_x * offset))
        result[index][1] = int(round(corners[index][1] - unit_y * offset))
    return result


def corners_center(corners):
    """返回四个角点的像素中心；角点缺失时返回 None。"""
    if corners is None:
        return None

    cx = sum(point[0] for point in corners) / 4.0
    cy = sum(point[1] for point in corners) / 4.0
    return [round(cx, 1), round(cy, 1)]


def roi_from_points(points, margin):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    x1 = int(clamp(min(xs) - margin, 0, IMAGE_W - 1))
    y1 = int(clamp(min(ys) - margin, 0, IMAGE_H - 1))
    x2 = int(clamp(max(xs) + margin, 1, IMAGE_W))
    y2 = int(clamp(max(ys) + margin, 1, IMAGE_H))

    return [x1, y1, x2 - x1, y2 - y1]


def green_blob_candidates(blobs, max_area):
    """提取全部合格绿光候选，按包围盒面积从大到小排列。"""
    candidates = []
    for blob in blobs:
        area = blob[2] * blob[3]
        if max_area is not None and area > max_area:
            continue
        center = [
            blob[0] + blob[2] // 2,
            blob[1] + blob[3] // 2,
        ]
        candidates.append([center, area])

    candidates.sort(key=lambda item: item[1], reverse=True)
    return candidates


def merge_green_candidates(candidates, merge_distance):
    """合并普通阈值和黑边阈值重复识别到的同一光斑。"""
    merged = []
    for candidate in candidates:
        center = candidate[0]
        area = candidate[1]
        duplicate_index = None
        for index, item in enumerate(merged):
            if point_distance(center, item[0]) <= merge_distance:
                duplicate_index = index
                break

        if duplicate_index is None:
            merged.append(candidate)
        elif area > merged[duplicate_index][1]:
            merged[duplicate_index] = candidate

    merged.sort(key=lambda item: item[1], reverse=True)
    return merged


def point_on_a4_black_border(point_px, homography, a4_mm_corners):
    """判断像素点是否落在A4纸外沿黑色胶布带状区域内。"""
    if point_px is None or homography is None or a4_mm_corners is None:
        return False

    point_mm = pixel_to_mm(point_px, homography)
    if point_mm is None:
        return False

    xs = [point[0] for point in a4_mm_corners]
    ys = [point[1] for point in a4_mm_corners]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    inside_outer = (
        min_x - BLACK_BORDER_OUTER_MARGIN_MM <= point_mm[0]
        <= max_x + BLACK_BORDER_OUTER_MARGIN_MM
        and min_y - BLACK_BORDER_OUTER_MARGIN_MM <= point_mm[1]
        <= max_y + BLACK_BORDER_OUTER_MARGIN_MM
    )
    if not inside_outer:
        return False

    return (
        point_mm[0] <= min_x + BLACK_BORDER_BAND_MM
        or point_mm[0] >= max_x - BLACK_BORDER_BAND_MM
        or point_mm[1] <= min_y + BLACK_BORDER_BAND_MM
        or point_mm[1] >= max_y - BLACK_BORDER_BAND_MM
    )


def green_candidates_are_halo_neighbors(candidate, strong_candidate, homography):
    """判断弱候选是否像同一激光的反射/光晕。"""
    center = candidate[0]
    strong_center = strong_candidate[0]
    close_in_px = (
        point_distance(center, strong_center)
        <= GREEN_HALO_SUPPRESS_DISTANCE_PX
    )

    close_in_mm = False
    if homography is not None:
        point_mm = pixel_to_mm(center, homography)
        strong_point_mm = pixel_to_mm(strong_center, homography)
        if point_mm is not None and strong_point_mm is not None:
            close_in_mm = (
                point_distance(point_mm, strong_point_mm)
                <= GREEN_HALO_SUPPRESS_DISTANCE_MM
            )

    return close_in_px or close_in_mm


def suppress_green_halo_candidates(candidates, homography):
    """保留主光斑，丢弃附近面积明显更小的反射/光晕候选。"""
    accepted = []
    for candidate in candidates:
        suppress = False
        for strong_candidate in accepted:
            if not green_candidates_are_halo_neighbors(
                candidate,
                strong_candidate,
                homography,
            ):
                continue
            if candidate[1] <= strong_candidate[1] * GREEN_HALO_AREA_RATIO:
                suppress = True
                break

        if not suppress:
            accepted.append(candidate)

    return accepted


def green_candidates_with_dual_lab(img, roi, homography, a4_mm_corners):
    """白色区域使用普通阈值；A4黑边区域额外使用黑边阈值。"""
    normal_blobs = img.find_blobs(
        GREEN_THRESHOLDS,
        roi=roi,
        pixels_threshold=BLOB_PIXELS_THRESHOLD,
        merge=True,
    )
    candidates = green_blob_candidates(
        normal_blobs,
        GREEN_MAX_BBOX_AREA,
    )

    if TARGET_MODE == "A4" and homography is not None and a4_mm_corners is not None:
        border_blobs = img.find_blobs(
            GREEN_BLACK_BORDER_THRESHOLDS,
            roi=roi,
            pixels_threshold=BLOB_PIXELS_THRESHOLD,
            merge=True,
        )
        border_candidates = green_blob_candidates(
            border_blobs,
            GREEN_MAX_BBOX_AREA,
        )
        for candidate in border_candidates:
            if point_on_a4_black_border(candidate[0], homography, a4_mm_corners):
                candidates.append(candidate)

    merged_candidates = merge_green_candidates(
        candidates,
        GREEN_CANDIDATE_MERGE_DISTANCE_PX,
    )
    return suppress_green_halo_candidates(
        merged_candidates,
        homography,
    )


def green_centers_are_separated(center1, center2, homography):
    """按实际毫米距离判断两束光是否超过最小间距；未标定时不确认。"""
    if center1 is None or center2 is None:
        return True

    if homography is None:
        return False

    point1_mm = pixel_to_mm(center1, homography)
    point2_mm = pixel_to_mm(center2, homography)
    if point1_mm is None or point2_mm is None:
        return False

    return point_distance(point1_mm, point2_mm) > GREEN_ROLE_MIN_SEPARATION_MM


def select_ordered_green_candidates(
    candidates,
    target_center,
    follower_center,
    pending_target_center=None,
    homography=None,
):
    """按上电锁定顺序和位置连续性给候选分配身份。

    target_center 为空时，面积最大的稳定候选用于确认第一束绿光。目标锁定
    后才选择与目标不同的第二束绿光。两束均锁定后，联合计算两两组合，
    选择相对上一帧总位移最小的身份分配，避免亮度变化导致角色互换。
    """
    if not candidates:
        return None, 0, None, 0

    if target_center is None:
        if pending_target_center is None:
            first_item = candidates[0]
        else:
            first_item = min(
                candidates,
                key=lambda item: point_distance(
                    item[0],
                    pending_target_center,
                ),
            )
        return first_item[0], first_item[1], None, 0

    if follower_center is None:
        target_index = min(
            range(len(candidates)),
            key=lambda i: point_distance(candidates[i][0], target_center),
        )
        target_item = candidates[target_index]
        if (
            point_distance(target_item[0], target_center)
            > GREEN_REACQUIRE_MAX_JUMP_PX
        ):
            target_item = [None, 0]
            target_index = None

        follower_item = [None, 0]
        target_reference = (
            target_item[0] if target_item[0] is not None else target_center
        )
        for index, item in enumerate(candidates):
            if index == target_index:
                continue
            if green_centers_are_separated(
                item[0], target_reference, homography
            ):
                follower_item = item
                break

        return (
            target_item[0], target_item[1],
            follower_item[0], follower_item[1],
        )

    best_pair = None
    best_score = 1e9
    second_best_score = 1e9
    for target_index, target_item in enumerate(candidates):
        target_distance = point_distance(target_item[0], target_center)
        if target_distance > GREEN_REACQUIRE_MAX_JUMP_PX:
            continue
        for follower_index, follower_item in enumerate(candidates):
            if follower_index == target_index:
                continue
            if not green_centers_are_separated(
                target_item[0], follower_item[0], homography
            ):
                continue
            follower_distance = point_distance(follower_item[0], follower_center)
            if follower_distance > GREEN_REACQUIRE_MAX_JUMP_PX:
                continue
            score = target_distance + follower_distance
            if score < best_score:
                second_best_score = best_score
                best_score = score
                best_pair = [target_item, follower_item]
            elif score < second_best_score:
                second_best_score = score

    if best_pair is not None:
        target_item = best_pair[0]
        follower_item = best_pair[1]
        target_other_distance = point_distance(
            target_item[0], follower_center
        )
        follower_other_distance = point_distance(
            follower_item[0], target_center
        )
        crossed_roles = (
            target_other_distance + GREEN_ROLE_CROSSOVER_MARGIN_PX
            < point_distance(target_item[0], target_center)
            and follower_other_distance + GREEN_ROLE_CROSSOVER_MARGIN_PX
            < point_distance(follower_item[0], follower_center)
        )

        if crossed_roles:
            return target_center, 0, follower_center, 0

        # 近距离或交叉时，正反两种身份分配的代价几乎相同；此时冻结
        # 身份一个帧，等待后续帧提供足够的位移差，而不是立即交换角色。
        if (
            second_best_score < 1e9
            and second_best_score - best_score
            <= GREEN_ASSIGNMENT_AMBIGUITY_MARGIN_PX
        ):
            return target_center, 0, follower_center, 0
        return (
            best_pair[0][0], best_pair[0][1],
            best_pair[1][0], best_pair[1][1],
        )

    # 仅剩一个可见光点时，把它分配给距离更近的已锁定角色，另一个角色
    # 依靠 BLOB_HOLD_FRAMES 暂时保持上一位置。
    best_single = None
    best_single_score = 1e9
    target_single_distance = None
    follower_single_distance = None
    for item in candidates:
        target_distance = point_distance(item[0], target_center)
        if target_distance <= GREEN_REACQUIRE_MAX_JUMP_PX:
            if (
                target_single_distance is None
                or target_distance < target_single_distance
            ):
                target_single_distance = target_distance
        if (
            target_distance <= GREEN_REACQUIRE_MAX_JUMP_PX
            and target_distance < best_single_score
        ):
                best_single = ["target", item]
                best_single_score = target_distance

        follower_distance = point_distance(item[0], follower_center)
        if follower_distance <= GREEN_REACQUIRE_MAX_JUMP_PX:
            if (
                follower_single_distance is None
                or follower_distance < follower_single_distance
            ):
                follower_single_distance = follower_distance
        if (
            green_centers_are_separated(item[0], target_center, homography)
            and
            follower_distance <= GREEN_REACQUIRE_MAX_JUMP_PX
            and follower_distance < best_single_score
        ):
                best_single = ["follower", item]
                best_single_score = follower_distance

    # 合并成一个候选光斑且同时接近两种角色时，不凭单帧距离猜身份。
    if (
        target_single_distance is not None
        and follower_single_distance is not None
        and abs(target_single_distance - follower_single_distance)
        <= GREEN_ASSIGNMENT_AMBIGUITY_MARGIN_PX
    ):
        return target_center, 0, follower_center, 0

    if best_single is None:
        return None, 0, None, 0
    if best_single[0] == "target":
        return best_single[1][0], best_single[1][1], None, 0
    return None, 0, best_single[1][0], best_single[1][1]


def is_a4_shape(corners):
    """用相邻边长度比排除明显的正方形和细长假矩形。"""
    lengths = []
    for i in range(4):
        lengths.append(point_distance(corners[i], corners[(i + 1) % 4]))

    side_a = (lengths[0] + lengths[2]) / 2.0
    side_b = (lengths[1] + lengths[3]) / 2.0
    short_side = min(side_a, side_b)
    long_side = max(side_a, side_b)

    if short_side < 1.0:
        return False

    ratio = long_side / short_side
    return 1.15 <= ratio <= 1.70


def a4_mm_corners_for_shape(corners):
    """按检测到的长边方向返回(A4毫米角点, 摆放方向)。"""
    top_bottom = (
        point_distance(corners[0], corners[1])
        + point_distance(corners[2], corners[3])
    ) / 2.0
    left_right = (
        point_distance(corners[1], corners[2])
        + point_distance(corners[3], corners[0])
    ) / 2.0

    if top_bottom >= left_right:
        return A4_MM_CORNERS_LANDSCAPE, "landscape"
    return A4_MM_CORNERS_PORTRAIT, "portrait"


def find_largest_a4_rect(img, roi):
    """返回画面中黑色边框矩形的四个顶点；找不到时返回 None。"""
    rects = img.find_rects(roi=roi, threshold=RECT_THRESHOLD)
    best_corners = None
    best_area = 0

    for rect in rects:
        raw_corners = rect.corners()
        corners = order_corners_clockwise(raw_corners)
        xs = [point[0] for point in raw_corners]
        ys = [point[1] for point in raw_corners]
        bounding_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
        if bounding_area < RECT_MIN_BOUNDING_AREA:
            continue

        if not is_a4_shape(corners):
            continue

        if bounding_area > best_area:
            best_area = bounding_area
            best_corners = corners

    return best_corners


def corners_are_close(corners1, corners2, tolerance):
    if corners1 is None or corners2 is None:
        return False

    for i in range(4):
        if point_distance(corners1[i], corners2[i]) > tolerance:
            return False
    return True


# 4. 单应变换：像素坐标转换为屏幕毫米坐标

def solve_linear_system(matrix, vector):
    """高斯消元求解8元一次方程，仅在程序启动时用于标定。"""
    n = len(vector)
    augmented = []
    for row_index in range(n):
        augmented.append(
            [float(value) for value in matrix[row_index]]
            + [float(vector[row_index])]
        )

    for column in range(n):
        pivot = max(range(column, n), key=lambda r: abs(augmented[r][column]))
        if abs(augmented[pivot][column]) < 1e-9:
            raise ValueError("标定点无效：无法计算像素到毫米变换")

        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]

        divisor = augmented[column][column]
        for j in range(column, n + 1):
            augmented[column][j] /= divisor

        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            for j in range(column, n + 1):
                augmented[row][j] -= factor * augmented[column][j]

    return [augmented[i][n] for i in range(n)]


def build_pixel_to_mm_homography(pixel_points, mm_points):
    matrix = []
    vector = []

    for index in range(4):
        u = float(pixel_points[index][0])
        v = float(pixel_points[index][1])
        x = float(mm_points[index][0])
        y = float(mm_points[index][1])

        matrix.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        vector.append(x)
        matrix.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        vector.append(y)

    return solve_linear_system(matrix, vector)


def pixel_to_mm(point, homography):
    if point is None or homography is None:
        return None

    u = float(point[0])
    v = float(point[1])
    denominator = homography[6] * u + homography[7] * v + 1.0

    if abs(denominator) < 1e-9:
        return None

    x = (homography[0] * u + homography[1] * v + homography[2]) / denominator
    y = (homography[3] * u + homography[4] * v + homography[5]) / denominator

    return [round(-x, 1), round(y, 1)]


def a4_rotation_angle(corners):
    """返回A4上边相对水平轴的旋转角（弧度）。"""
    if corners is None or len(corners) < 2:
        return 0.0
    dx = float(corners[1][0] - corners[0][0])
    dy = float(corners[1][1] - corners[0][1])
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0.0
    return math.atan2(dy, dx)


def rotate_mm_point(point, angle):
    """将毫米坐标绕原点旋转angle；坐标约定为x左正、y上正。"""
    if point is None or abs(angle) < 1e-9:
        return point
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return [
        round(point[0] * cos_angle - point[1] * sin_angle, 1),
        round(point[0] * sin_angle + point[1] * cos_angle, 1),
    ]


# pixel_to_mm() 已将 X 轴反向（输出坐标约定为 X 左正）。因此图像中
# 检测到的旋转角映射到毫米/下位机坐标系时需要取反，避免 Y 轴镜像。
def rotate_a4_output_point(point):
    return rotate_mm_point(point, -a4_rotation_angle_rad)


def expand_a4_vertical_edges_mm(
    vertices,
    left_outward_mm,
    right_outward_mm,
):
    """将A4左右竖边按各自实测误差向黑框外侧补偿。"""
    if vertices is None:
        return vertices

    adjusted = []
    for point in vertices:
        if point is None:
            adjusted.append(None)
            continue

        x = point[0]
        if x > 0.0:
            adjusted.append([
                round(x + left_outward_mm, 1),
                point[1],
            ])
        elif x < 0.0:
            adjusted.append([
                round(x - right_outward_mm, 1),
                point[1],
            ])
        else:
            adjusted.append([x, point[1]])

    return adjusted


# 5. 按Word约定生成紧凑JSON行

def build_vision_result_json(seq, valid, target_green_mm, vertices_mm):
    if not valid:
        return (
            '{"type":"VISION_RESULT","seq":%d,"valid":0,'
            '"red_spot":{"x_mm":0.0,"y_mm":0.0},'
            '"vertex_count":0,"vertices_mm":[]}\r\n'
        ) % seq

    return (
        '{"type":"VISION_RESULT","seq":%d,"valid":1,'
        '"red_spot":{"x_mm":%.1f,"y_mm":%.1f},'
        '"vertex_count":4,"vertices_mm":['
        '[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]}\r\n'
    ) % (
        seq,
        target_green_mm[0], target_green_mm[1],
        vertices_mm[0][0], vertices_mm[0][1],
        vertices_mm[1][0], vertices_mm[1][1],
        vertices_mm[2][0], vertices_mm[2][1],
        vertices_mm[3][0], vertices_mm[3][1],
    )


def build_track_result_json(seq, valid, target_green_mm, follower_green_mm):
    """保持旧字段名：red_spot 是目标，green_spot 是追踪点。"""
    if not valid:
        return (
            '{"type":"TRACK_RESULT","seq":%d,"valid":0,'
            '"red_spot":{"x_mm":0.0,"y_mm":0.0},'
            '"green_spot":{"x_mm":0.0,"y_mm":0.0}}\r\n'
        ) % seq

    return (
        '{"type":"TRACK_RESULT","seq":%d,"valid":1,'
        '"red_spot":{"x_mm":%.1f,"y_mm":%.1f},'
        '"green_spot":{"x_mm":%.1f,"y_mm":%.1f}}\r\n'
    ) % (
        seq,
        target_green_mm[0], target_green_mm[1],
        follower_green_mm[0], follower_green_mm[1],
    )


def send_json_line(serial_port, json_line):
    # Word协议限制单帧不超过256字节。
    global last_uart_error_ms

    encoded = json_line.encode("utf-8")
    if len(encoded) > 256:
        now_ms = time.ticks_ms()
        if now_ms - last_uart_error_ms >= UART_ERROR_LOG_INTERVAL_MS:
            last_uart_error_ms = now_ms
            print("ERROR: JSON line too long:", len(encoded))
        return -1

    if serial_port is None:
        return 0

    try:
        written = serial_port.write(encoded)
    except Exception as error:
        now_ms = time.ticks_ms()
        if now_ms - last_uart_error_ms >= UART_ERROR_LOG_INTERVAL_MS:
            last_uart_error_ms = now_ms
            print("UART1 WRITE ERROR:", error)
        return -1

    if written != len(encoded):
        now_ms = time.ticks_ms()
        if now_ms - last_uart_error_ms >= UART_ERROR_LOG_INTERVAL_MS:
            last_uart_error_ms = now_ms
            print("UART1 SHORT WRITE:", written, "/", len(encoded))

    return written


# 6. 画面调试标记

def draw_polygon(img, corners, color):
    if corners is None:
        return

    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        img.draw_line(p1[0], p1[1], p2[0], p2[1], color=color, thickness=1)
        img.draw_circle(p1[0], p1[1], radius=3, color=color, thickness=-1)


def init_uart_port():
    """初始化MaixCAM-Pro UART1；关闭串口时返回None。"""
    if not ENABLE_UART:
        print("UART1 DISABLED: set ENABLE_UART=True to send JSON.")
        return None

    rx_map_result = pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNCTION)
    err.check_raise(
        rx_map_result,
        "MaixCAM-Pro failed to map A18 as UART1_RX",
    )

    tx_map_result = pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION)
    err.check_raise(
        tx_map_result,
        "MaixCAM-Pro failed to map A19 as UART1_TX",
    )

    serial_port = uart.UART(UART_DEVICE, UART_BAUDRATE)
    if not serial_port.is_open():
        raise RuntimeError("MaixCAM-Pro UART1 failed to open")

    print(
        "MaixCAM-Pro UART1 READY:",
        "open=", serial_port.is_open(),
        "device=", UART_DEVICE,
        "baud=", UART_BAUDRATE,
        "RX=", [UART_RX_PIN, UART_RX_FUNCTION],
        "TX=", [UART_TX_PIN, UART_TX_FUNCTION],
        "send_mode=AUTOMATIC",
    )
    return serial_port


def init_camera_and_display():
    """初始化相机参数和屏幕，返回(camera, display)。"""
    camera_device = camera.Camera(IMAGE_W, IMAGE_H)
    camera_device.exp_mode(camera.AeMode.Auto)
    camera_device.awb_mode(camera.AwbMode.Auto)
    camera_device.luma(60)
    camera_device.constrast(50)
    camera_device.saturation(75)
    camera_device.skip_frames(60)

    return camera_device, display.Display()


# 7. 硬件与标定初始化

serial1 = init_uart_port()
cam, disp = init_camera_and_display()

print("ad.py VERSION:", PROGRAM_VERSION)

if TARGET_MODE == "A4" and ENABLE_LENS_CORR:
    print(
        "LENS_CORR ENABLED: strength=",
        LENS_CORR_STRENGTH,
        "zoom=",
        LENS_CORR_ZOOM,
    )

homography = None

if CALIBRATION_READY:
    # 白框保持原始大小：A4/SQUARE 都使用固定正方形角点外扩的搜索ROI。
    tracking_roi = roi_from_points(
        CALIBRATION_PIXEL_CORNERS,
        TRACKING_ROI_MARGIN,
    )
else:
    tracking_roi = [0, 0, IMAGE_W, IMAGE_H]

if TARGET_MODE == "SQUARE" and CALIBRATION_READY:
    homography = build_pixel_to_mm_homography(
        CALIBRATION_PIXEL_CORNERS,
        CALIBRATION_MM_CORNERS,
    )
    print("SQUARE calibration ready. tracking_roi=", tracking_roi)
elif not CALIBRATION_READY:
    print("WARNING: CALIBRATION_READY=False; JSON valid will stay 0.")
else:
    # A4模式：标定来自运行时检测到的黑框四角；搜索ROI保持原始白框大小。
    print("A4 mode: detecting black border corners; fixed calibration not used.")


# 8. 运行状态
a4_corners_px = None
a4_calib_corners_px = None
a4_reference_mm_corners = None
a4_rotation_angle_rad = 0.0
pending_a4_corners = None
pending_a4_count = 0
pending_a4_corner_sum = None

target_green_center_px = None
follower_green_center_px = None
target_green_missed = BLOB_HOLD_FRAMES + 1
follower_green_missed = BLOB_HOLD_FRAMES + 1

pending_target_green_center = None
pending_target_green_count = 0
pending_follower_green_center = None
pending_follower_green_count = 0

# 重合期间的身份防交换状态。
overlap_active = False
overlap_anchor_px = None
overlap_first_mover_locked = False

# 整体运动方向的防抖状态。
motion_previous_centroid_px = None
motion_direction_px = None
motion_direction_count = 0

origin_target_green_px = None
origin_target_green_mm = None

frame_count = 0
vision_seq = 0
track_seq = 0
last_send_ms = time.ticks_ms() - SEND_INTERVAL_MS
last_uart_error_ms = 0
last_vision_json_bytes = 0
last_track_json_bytes = 0
last_vision_uart_bytes = 0
last_track_uart_bytes = 0


# 9. 主循环

while not app.need_exit():
    img = cam.read()
    # A4模式先做镜头畸变校正，A4角点和两束绿光都在校正后的图像坐标上处理。
    if TARGET_MODE == "A4" and ENABLE_LENS_CORR:
        try:
            img = img.lens_corr(
                strength=LENS_CORR_STRENGTH,
                zoom=LENS_CORR_ZOOM,
                x_corr=LENS_CORR_X,
                y_corr=LENS_CORR_Y,
            )
        except Exception as error:
            print("LENS_CORR FAILED:", error, "disabled for this run.")
            ENABLE_LENS_CORR = False
    # 绘制都在检测之后进行，直接复用当前帧，避免额外拷贝。
    view = img

    # A4模式：连续多帧确认后锁定。更换A4位置后重新运行程序。
    if TARGET_MODE == "A4" and a4_corners_px is None:
        candidate = find_largest_a4_rect(img, tracking_roi)

        if candidate is not None:
            if corners_are_close(
                candidate,
                pending_a4_corners,
                RECT_STABLE_TOLERANCE_PX,
            ):
                pending_a4_count += 1
                for i in range(4):
                    pending_a4_corner_sum[i][0] += candidate[i][0]
                    pending_a4_corner_sum[i][1] += candidate[i][1]
            else:
                pending_a4_corners = candidate
                pending_a4_count = 1
                pending_a4_corner_sum = [
                    [float(point[0]), float(point[1])] for point in candidate
                ]

            if pending_a4_count >= RECT_CONFIRM_FRAMES:
                # 用连续确认帧的平均角点建立标定，降低单帧抖动误差；
                # 输出轨迹角点再按 RECT_PATH_SCALE 缩放。
                a4_calib_corners_px = [
                    [
                        int(round(pending_a4_corner_sum[i][0] / pending_a4_count)),
                        int(round(pending_a4_corner_sum[i][1] / pending_a4_count)),
                    ]
                    for i in range(4)
                ]
                if CALIBRATION_READY:
                    a4_mm_corners, a4_orientation = a4_mm_corners_for_shape(
                        a4_calib_corners_px,
                    )
                    a4_reference_mm_corners = a4_mm_corners
                    if PRESERVE_A4_ROTATION:
                        a4_rotation_angle_rad = a4_rotation_angle(
                            a4_calib_corners_px,
                        )
                    else:
                        a4_rotation_angle_rad = 0.0
                    homography = build_pixel_to_mm_homography(
                        a4_calib_corners_px,
                        a4_mm_corners,
                    )
                else:
                    a4_orientation = None
                a4_corners_px = raise_bottom_edge(
                    lower_top_edge(
                        scale_corners(a4_calib_corners_px, RECT_PATH_SCALE),
                        RECT_TOP_LOWER_RATIO,
                    ),
                    RECT_BOTTOM_RAISE_RATIO,
                )
                print(
                    "A4 black border locked, orientation=",
                    a4_orientation,
                    "calib_px=",
                    a4_calib_corners_px,
                    "rotation_deg=",
                    round(a4_rotation_angle_rad * 180.0 / math.pi, 1),
                    "path_px=",
                    a4_corners_px,
                )
        else:
            pending_a4_corners = None
            pending_a4_count = 0
            pending_a4_corner_sum = None

    # 白色区域使用普通绿光阈值；A4黑胶带边框区域额外启用黑边阈值。
    # 所有绿光最终仍统一进入同一个候选集合，身份只由上电后的锁定顺序决定。
    green_candidates = green_candidates_with_dual_lab(
        img,
        tracking_roi,
        homography,
        a4_reference_mm_corners,
    )

    # 两点已经重合时，不立即按最近邻重新分配身份。记录重合中心；
    # 后续重新出现两个候选点时，先离开该中心的点固定为被追踪目标。
    previous_target_green = target_green_center_px
    previous_follower_green = follower_green_center_px
    overlap_match = False
    if previous_target_green is not None and previous_follower_green is not None:
        if point_distance(previous_target_green, previous_follower_green) <= GREEN_OVERLAP_MAX_DISTANCE_PX:
            overlap_match = True
    if len(green_candidates) == 1 and previous_target_green is not None and previous_follower_green is not None:
        merged_center = green_candidates[0][0]
        if (
            point_distance(merged_center, previous_target_green) <= GREEN_TRACK_TOLERANCE_PX
            and point_distance(merged_center, previous_follower_green) <= GREEN_TRACK_TOLERANCE_PX
        ):
            overlap_match = True

    if overlap_match and not overlap_active:
        if len(green_candidates) == 1:
            overlap_anchor_px = list(green_candidates[0][0])
        else:
            overlap_anchor_px = [
                int(round((previous_target_green[0] + previous_follower_green[0]) / 2.0)),
                int(round((previous_target_green[1] + previous_follower_green[1]) / 2.0)),
            ]
        overlap_active = True
        overlap_first_mover_locked = False
        print("GREEN OVERLAP LOCK START:", overlap_anchor_px)

    overlap_override = False
    if (
        overlap_active
        and not overlap_first_mover_locked
        and overlap_anchor_px is not None
        and len(green_candidates) >= 2
    ):
        # 不能用像素距离代替实际距离；只有建立单应标定后才能判断1 cm。
        overlap_anchor_mm = pixel_to_mm(overlap_anchor_px, homography)
        movement_candidates = []
        if overlap_anchor_mm is not None:
            for index, item in enumerate(green_candidates):
                if not all(
                    green_centers_are_separated(item[0], other[0], homography)
                    for other_index, other in enumerate(green_candidates)
                    if other_index != index
                ):
                    continue
                item_mm = pixel_to_mm(item[0], homography)
                if item_mm is not None:
                    movement_candidates.append(
                        (item, point_distance(item_mm, overlap_anchor_mm))
                    )

        if len(movement_candidates) >= 2:
            movement_candidates.sort(key=lambda pair: pair[1], reverse=True)
            first_item, first_move_mm = movement_candidates[0]
            second_item, second_move_mm = movement_candidates[1]

            # 只有某一束达到1 cm才触发锁定；两点同时达到时要求额外的
            # 位移差，仍不足时保持原身份等待下一帧。
            if (
                first_move_mm >= GREEN_OVERLAP_FIRST_MOVE_THRESHOLD_MM
                and (
                    second_move_mm < GREEN_OVERLAP_FIRST_MOVE_THRESHOLD_MM
                    or first_move_mm - second_move_mm
                    >= GREEN_OVERLAP_FIRST_MOVE_MARGIN_MM
                )
            ):
                current_target_green = first_item[0]
                target_green_area = first_item[1]
                current_follower_green = second_item[0]
                follower_green_area = second_item[1]
                overlap_first_mover_locked = True
                overlap_active = False
                overlap_override = True
                print(
                    "GREEN OVERLAP FIRST MOVER LOCKED AS TARGET:",
                    current_target_green,
                    "follower=",
                    current_follower_green,
                    "moves=",
                    round(first_move_mm, 1),
                    round(second_move_mm, 1),
                    "mm",
                )

    if not overlap_override:
        (
            current_target_green,
            target_green_area,
            current_follower_green,
            follower_green_area,
        ) = select_ordered_green_candidates(
            green_candidates,
            target_green_center_px,
            follower_green_center_px,
            pending_target_green_center,
            homography,
        )

    # 运动方向判别：用两束光的中心点估计整体运动向量，再比较两点在
    # 该方向上的投影。投影更靠前的光点判定为被追踪目标，后面的为追踪光。
    # 仅在连续若干帧方向稳定且前后距离足够大时覆盖最近邻结果。
    if len(green_candidates) >= 2:
        centroid_x = sum(item[0][0] for item in green_candidates[:2]) / 2.0
        centroid_y = sum(item[0][1] for item in green_candidates[:2]) / 2.0
        current_centroid_px = [centroid_x, centroid_y]
        if motion_previous_centroid_px is not None:
            motion_dx = centroid_x - motion_previous_centroid_px[0]
            motion_dy = centroid_y - motion_previous_centroid_px[1]
            motion_speed = math.sqrt(motion_dx * motion_dx + motion_dy * motion_dy)
            if motion_speed >= GREEN_DIRECTION_SPEED_THRESHOLD_PX:
                if motion_direction_px is not None:
                    dot = (
                        motion_dx * motion_direction_px[0]
                        + motion_dy * motion_direction_px[1]
                    )
                    if dot < 0:
                        motion_direction_count = 1
                    else:
                        motion_direction_count += 1
                else:
                    motion_direction_count = 1
                motion_direction_px = [motion_dx, motion_dy]
            else:
                motion_direction_count = max(0, motion_direction_count - 1)
        motion_previous_centroid_px = current_centroid_px

        if (
            current_target_green is not None
            and current_follower_green is not None
            and motion_direction_px is not None
            and motion_direction_count >= GREEN_DIRECTION_CONFIRM_FRAMES
        ):
            direction_length = math.sqrt(
                motion_direction_px[0] * motion_direction_px[0]
                + motion_direction_px[1] * motion_direction_px[1]
            )
            if direction_length > 0:
                dir_x = motion_direction_px[0] / direction_length
                dir_y = motion_direction_px[1] / direction_length
                target_projection = (
                    current_target_green[0] * dir_x
                    + current_target_green[1] * dir_y
                )
                follower_projection = (
                    current_follower_green[0] * dir_x
                    + current_follower_green[1] * dir_y
                )
                front_margin = abs(target_projection - follower_projection)
                if front_margin >= GREEN_DIRECTION_FRONT_MARGIN_PX:
                    if follower_projection > target_projection:
                        current_target_green, current_follower_green = (
                            current_follower_green,
                            current_target_green,
                        )
                        target_green_area, follower_green_area = (
                            follower_green_area,
                            target_green_area,
                        )
                    print(
                        "GREEN DIRECTION ROLE LOCK:",
                        "target(front)=",
                        current_target_green,
                        "follower(back)=",
                        current_follower_green,
                        "direction=",
                        [round(dir_x, 2), round(dir_y, 2)],
                        "front_margin=",
                        round(front_margin, 1),
                    )
    elif motion_direction_count > 0:
        motion_direction_count -= 1

    # 第一束绿光连续稳定出现后锁为被追踪目标。锁定后，小位移直接续锁；
    # 大位移需要再次连续确认，减少反光点造成的身份跳变。
    if current_target_green is not None:
        target_green_missed = 0
        if (
            target_green_center_px is not None
            and point_distance(current_target_green, target_green_center_px)
            <= GREEN_TRACK_TOLERANCE_PX
        ):
            target_green_center_px = current_target_green
            pending_target_green_center = None
            pending_target_green_count = 0
        else:
            if (
                pending_target_green_center is None
                or point_distance(
                    current_target_green,
                    pending_target_green_center,
                ) > GREEN_STABLE_TOLERANCE_PX
            ):
                pending_target_green_center = current_target_green
                pending_target_green_count = 1
            else:
                pending_target_green_count += 1

            if pending_target_green_count >= FIRST_GREEN_CONFIRM_FRAMES:
                target_green_center_px = current_target_green
                target_green_missed = 0
                pending_target_green_center = None
                pending_target_green_count = 0
                if (
                    USE_FIRST_GREEN_AS_ORIGIN
                    and origin_target_green_px is None
                ):
                    origin_target_green_px = list(current_target_green)
                    print("FIRST GREEN TARGET ORIGIN PX:", origin_target_green_px)
                print("FIRST GREEN LOCKED AS TARGET:", target_green_center_px)
    else:
        target_green_missed += 1
        pending_target_green_center = None
        pending_target_green_count = 0

    # 只有第一束目标绿光锁定后，第二束候选才开始确认。这样身份由出现
    # 先后决定，而不是由光斑亮度决定。
    if target_green_center_px is not None and current_follower_green is not None:
        jump_distance = (
            point_distance(current_follower_green, follower_green_center_px)
            if follower_green_center_px is not None
            else 0
        )
        if (
            follower_green_center_px is not None
            and jump_distance <= GREEN_TRACK_TOLERANCE_PX
        ):
            follower_green_center_px = current_follower_green
            follower_green_missed = 0
            pending_follower_green_center = None
            pending_follower_green_count = 0
        else:
            if (
                pending_follower_green_center is None
                or point_distance(
                    current_follower_green,
                    pending_follower_green_center,
                ) > GREEN_STABLE_TOLERANCE_PX
            ):
                pending_follower_green_center = current_follower_green
                pending_follower_green_count = 1
            else:
                pending_follower_green_count += 1

            if pending_follower_green_count >= SECOND_GREEN_CONFIRM_FRAMES:
                first_lock = follower_green_center_px is None
                follower_green_center_px = current_follower_green
                follower_green_missed = 0
                pending_follower_green_center = None
                pending_follower_green_count = 0
                if first_lock:
                    print(
                        "SECOND GREEN LOCKED AS FOLLOWER:",
                        follower_green_center_px,
                    )
            else:
                follower_green_missed += 1
    else:
        follower_green_missed += 1
        pending_follower_green_center = None
        pending_follower_green_count = 0

    # 第一束目标绿光先于A4锁定出现时，等标定建立后再换算原点毫米坐标。
    if (
        USE_FIRST_GREEN_AS_ORIGIN
        and origin_target_green_px is not None
        and origin_target_green_mm is None
        and homography is not None
    ):
        origin_target_green_mm = pixel_to_mm(
            origin_target_green_px,
            homography,
        )
        if TARGET_MODE == "A4" and PRESERVE_A4_ROTATION:
            origin_target_green_mm = rotate_a4_output_point(
                origin_target_green_mm,
            )
        print(
            "FIRST GREEN TARGET ORIGIN SET: px=",
            origin_target_green_px,
            "mm=",
            origin_target_green_mm,
        )

    target_green_pixel_valid = target_green_center_px is not None
    follower_green_pixel_valid = (
        follower_green_center_px is not None
        and follower_green_missed <= BLOB_HOLD_FRAMES
    )

    target_green_mm = (
        pixel_to_mm(target_green_center_px, homography)
        if target_green_pixel_valid
        else None
    )
    follower_green_mm = (
        pixel_to_mm(follower_green_center_px, homography)
        if follower_green_pixel_valid
        else None
    )

    # 红色目标顶点：0.5m正方形用固定毫米坐标；A4用检测角点转换。
    if TARGET_MODE == "SQUARE":
        target_vertices_mm = CALIBRATION_MM_CORNERS if CALIBRATION_READY else None
        target_vertices_px = CALIBRATION_PIXEL_CORNERS if CALIBRATION_READY else None
    else:
        target_vertices_px = a4_corners_px
        if CALIBRATION_READY and a4_corners_px is not None:
            target_vertices_mm = [
                pixel_to_mm(point, homography) for point in a4_corners_px
            ]
            target_vertices_mm = expand_a4_vertical_edges_mm(
                target_vertices_mm,
                A4_LEFT_EDGE_OUTWARD_MM,
                A4_RIGHT_EDGE_OUTWARD_MM,
            )
        else:
            target_vertices_mm = None

    # 单应变换默认把A4拉正为水平/竖直矩形；这里将所有输出坐标统一
    # 旋回检测到的黑框角度，使下位机执行斜着的边框轨迹。
    if TARGET_MODE == "A4" and PRESERVE_A4_ROTATION:
        target_green_mm = rotate_a4_output_point(target_green_mm)
        follower_green_mm = rotate_a4_output_point(follower_green_mm)
        if target_vertices_mm is not None:
            target_vertices_mm = [
                rotate_a4_output_point(point)
                for point in target_vertices_mm
            ]

    if (
        USE_FIRST_GREEN_AS_ORIGIN
        and origin_target_green_mm is not None
    ):
        if target_green_mm is not None:
            target_green_mm = [
                round(target_green_mm[0] - origin_target_green_mm[0], 1),
                round(target_green_mm[1] - origin_target_green_mm[1], 1),
            ]
        if follower_green_mm is not None:
            follower_green_mm = [
                round(follower_green_mm[0] - origin_target_green_mm[0], 1),
                round(follower_green_mm[1] - origin_target_green_mm[1], 1),
            ]
        if target_vertices_mm is not None:
            target_vertices_mm = [
                [
                    round(point[0] - origin_target_green_mm[0], 1),
                    round(point[1] - origin_target_green_mm[1], 1),
                ]
                for point in target_vertices_mm
            ]

    sent_target_mm = target_green_mm
    if sent_target_mm is None and FORCE_VISION_VALID:
        sent_target_mm = TARGET_FALLBACK_MM

    vision_valid = (
        CALIBRATION_READY
        and sent_target_mm is not None
        and target_vertices_mm is not None
        and len(target_vertices_mm) == 4
        and all(point is not None for point in target_vertices_mm)
    )
    track_valid = (
        CALIBRATION_READY
        and target_green_mm is not None
        and follower_green_mm is not None
    )

    # 生成并发送两个独立JSON对象，每个对象一行。
    now_ms = time.ticks_ms()
    if now_ms - last_send_ms >= SEND_INTERVAL_MS:
        last_send_ms = now_ms
        if ENABLE_VISION_RESULT:
            vision_json = build_vision_result_json(
                vision_seq,
                1 if vision_valid else 0,
                sent_target_mm,
                target_vertices_mm,
            )
            last_vision_uart_bytes = send_json_line(serial1, vision_json)
            last_vision_json_bytes = len(vision_json.encode("utf-8"))
            vision_seq = (vision_seq + 1) & 0xFFFFFFFF
        track_json = build_track_result_json(
            track_seq,
            1 if track_valid else 0,
            target_green_mm,
            follower_green_mm,
        )
        last_track_uart_bytes = send_json_line(serial1, track_json)
        last_track_json_bytes = len(track_json.encode("utf-8"))
        track_seq = (track_seq + 1) & 0xFFFFFFFF

    frame_count += 1

    # 画面显示：第一束目标用青色，第二束追踪点用亮绿色。
    view.draw_rect(
        tracking_roi[0], tracking_roi[1],
        tracking_roi[2], tracking_roi[3],
        color=image.Color.from_rgb(255, 255, 255),
        thickness=1,
    )

    if target_vertices_px is not None:
        draw_polygon(
            view,
            [[int(p[0]), int(p[1])] for p in target_vertices_px],
            image.Color.from_rgb(255, 255, 0),
        )

    if target_green_pixel_valid and target_green_center_px is not None:
        view.draw_circle(
            target_green_center_px[0], target_green_center_px[1], radius=4,
            color=image.Color.from_rgb(0, 255, 255), thickness=-1,
        )

    if follower_green_pixel_valid and follower_green_center_px is not None:
        view.draw_circle(
            follower_green_center_px[0], follower_green_center_px[1], radius=4,
            color=image.Color.from_rgb(0, 255, 0), thickness=-1,
        )

    if frame_count % DEBUG_PRINT_EVERY == 0:
        print(
            "mode=", TARGET_MODE,
            "a4_calib_px=", a4_calib_corners_px,
            "a4_center_px=", corners_center(a4_calib_corners_px),
            "a4_px=", a4_corners_px,
            "role_state=", (
                "tracking"
                if follower_green_center_px is not None
                else "waiting_follower"
                if target_green_center_px is not None
                else "waiting_target"
            ),
            "origin_px=", origin_target_green_px,
            "origin_mm=", origin_target_green_mm,
            "target_green_px=", (
                target_green_center_px if target_green_pixel_valid else None
            ),
            "target_green_missed=", target_green_missed,
            "follower_green_px=", (
                follower_green_center_px if follower_green_pixel_valid else None
            ),
            "follower_green_missed=", follower_green_missed,
            "target_green_mm=", target_green_mm,
            "follower_green_mm=", follower_green_mm,
            "vertices_mm=", target_vertices_mm,
            "vision_valid=", vision_valid,
            "track_valid=", track_valid,
            "target_green_area=", target_green_area,
            "follower_green_area=", follower_green_area,
            "json_bytes=", [last_vision_json_bytes, last_track_json_bytes],
            "uart_bytes=", [last_vision_uart_bytes, last_track_uart_bytes],
        )

    disp.show(view)
