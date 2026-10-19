"""Projection/rasterization helpers shared with the previous viewer.
All poses and map points supplied here come from ORB-SLAM3.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass
class Camera:
    # dataclass 自动生成初始化方法；Camera(...) 就能把下面这些字段装成一个对象。
    # id 是相机内参组的编号，多张图像可以共享它，并不代表“第几帧”。
    id: int
    # model 决定 params 的排列方式，以及需要采用哪一种镜头畸变公式。
    model: str
    # 这是 ORB-SLAM3 输入图像的宽高，不一定等于输出演示视频的宽高。
    width: int
    height: int
    # 内参数组：焦距 f 或 fx/fy，主点 cx/cy，以及模型需要的畸变系数。
    params: np.ndarray

    def project(self, pc: np.ndarray) -> np.ndarray:
        """Project camera coordinates onto the ORIGINAL (distorted) image."""
        # 输入 pc：N×3 相机坐标，每行 [X, Y, Z]。
        # 输出：N×2 原图像素坐标，每行 [u, v]，这里会保留原镜头的畸变。
        # 本方法只负责计算，调用者还需筛掉 Z<=0、无效值和图像范围以外的点。
        # 暂时关闭除零等数值警告；坏点仍会产生 inf/NaN，后续通过筛选排除。
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            # 透视除法：x=X/Z、y=Y/Z。相同大小的物体，距离越远，投影越小。
            xy = pc[:, :2] / pc[:, 2:3]
            p = self.params
            # SIMPLE 系列和 RADIAL 共用一个焦距，其余支持的模型分别给出 fx、fy。
            if self.model in {"SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"}:
                fx = fy = p[0]
                cx, cy = p[1:3]
            else:
                fx, fy, cx, cy = p[:4]
            # r²=x²+y²：到光轴的归一化距离平方，不是三维点到相机的距离。
            r2 = np.sum(xy * xy, axis=1)
            if self.model == "SIMPLE_RADIAL":
                # 一项径向畸变：离图像中心越远，系数 k 对位置的影响通常越大。
                xy = xy * (1 + p[3] * r2[:, None])
            elif self.model == "RADIAL":
                # 两项径向畸变：缩放系数为 1+k1*r²+k2*r⁴。
                xy = xy * (1 + p[3] * r2 + p[4] * r2**2)[:, None]
            elif self.model in {"OPENCV", "FULL_OPENCV"}:
                x, y = xy.T
                # k1/k2 描述径向畸变；p1/p2 描述镜头偏心等造成的切向畸变。
                k1, k2, p1, p2 = p[4:8]
                radial = 1 + k1 * r2 + k2 * r2**2
                if self.model == "FULL_OPENCV":
                    # FULL_OPENCV 再加四个系数，使用“分子/分母”的径向模型。
                    k3, k4, k5, k6 = p[8:12]
                    radial = (radial + k3 * r2**3) / (
                        1 + k4 * r2 + k5 * r2**2 + k6 * r2**3)
                # 分别修正 x 和 y，再拼回 N×2 数组；PINHOLE 模型无需畸变修正。
                xy = np.column_stack((
                    x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x*x),
                    y * radial + p1 * (r2 + 2 * y*y) + 2 * p2 * x * y))
            # 焦距把归一化坐标换算为像素，主点把原点移到图像中的光轴位置。
            return xy * [fx, fy] + [cx, cy]


@dataclass
class Pose:
    # 一张已注册图像的位姿与观测。图像 id 是标识符，不是时间顺序。
    id: int
    q: np.ndarray  # Hamilton 四元数，顺序 w/x/y/z，表示世界坐标 -> 相机坐标的旋转。
    # 长度为 3 的平移向量，参与 Pc=R@Pw+t；不是相机的世界位置。
    t: np.ndarray
    # 根据这个编号，从 Model.cameras 中找到本图像使用的内参。
    camera_id: int
    name: str
    observations: np.ndarray  # M×3，每行是像素 x/y 和三维点 ID；-1 表示未关联三维点。

    @property
    def R(self):
        # @property 允许通过 pose.R 访问计算结果，不必写 pose.R()。
        # 四元数是紧凑的朝向表示，旋转矩阵则方便直接变换三维点。
        return q_to_R(self.q)

    @property
    def center(self):
        # 相机中心在自身坐标系中是 0，所以 0=R@C+t，解得 C=-R.T@t。
        # 旋转矩阵的逆等于其转置；这里返回可用于绘制轨迹的世界位置。
        return -self.R.T @ self.t


@dataclass
class Model:
    # 整个模型的数据容器。以下与点有关的数组，行号一一对应：
    # ids[k]、xyz[k]、bgr[k]、birth[k] 都描述同一个三维点。
    cameras: dict[int, Camera]
    poses: list[Pose]  # 按图像文件名的自然顺序排序后的位姿列表。
    ids: np.ndarray  # N 个三维点 ID；ID 不一定连续，也不等于数组行号。
    xyz: np.ndarray  # N×3 世界坐标。
    bgr: np.ndarray  # N×3 固定地图颜色，每个通道为 0~255。
    birth: np.ndarray  # N 个最早保留观测的时间（秒），用于逐步显示最终地图。


def q_to_R(q):
    # 将 [w, x, y, z] 四元数转换为 3×3 世界 -> 相机旋转矩阵。
    # float64 使用双精度浮点数，减少多次几何运算带来的数值误差。
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    # 四元数必须能归一化；长度接近零或不是有限数时无法表示有效朝向。
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion")
    w, x, y, z = q / norm
    # 下面是单位四元数到旋转矩阵的展开公式，不是对三个轴分别乘一个角度。
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def splat(uv, depths, colors, width, height, radius=0, background=(15, 17, 14)):
    """Small circular point sprites, with a per-pixel nearest-depth buffer."""
    # 把投影后的点画成小圆点。输入：N×2 像素位置、N 个深度、N×3 BGR 颜色。
    # 返回 (图像, 实际被点覆盖的像素数)；多个点可能覆盖同一像素，所以不是点数。
    # radius 是显示半径：0 表示单像素，增大它只让点更醒目，不增加三维细节。
    canvas = np.empty((height, width, 3), np.uint8)
    canvas[:] = background
    # zbuf 是深度缓冲：记录每个像素目前画出的最近深度，初始为无穷远。
    # 例如深度为 2 的点和深度为 5 的点重叠，应显示深度为 2 的近点。
    zbuf = np.full(width*height, np.inf)
    # 把图像看成一串像素，二维 (x,y) 对应一维编号 y*width+x。
    # reshape 得到的视图与 canvas 共用数据，在 flat 上写颜色会直接更新图像。
    flat = canvas.reshape(-1, 3)
    finite = np.isfinite(uv).all(axis=1) & np.isfinite(depths) & (depths > 1e-9)
    # 先在浮点数阶段去掉远离画面的坐标，再转整数，防止极大值转换时溢出。
    # 边缘留出 radius 的空间，让圆点中心在画面外时仍能画出进入画面的部分。
    finite &= (uv[:, 0] >= -radius-.5) & (uv[:, 0] < width+radius)
    finite &= (uv[:, 1] >= -radius-.5) & (uv[:, 1] < height+radius)
    uv, depths, colors = uv[finite], depths[finite], colors[finite]
    # 从近到远排序；stable 让相同深度的点保留原来的先后顺序。
    order = np.argsort(depths, kind="stable")
    # rint 取最近整数像素位置，然后转换为整数下标；恰好半整数时取相邻偶数。
    pixels = np.rint(uv[order]).astype(np.int64)
    depths, colors = depths[order], colors[order]
    # 遍历圆形小点内的像素偏移，例如半径 1 会覆盖中心和上下左右。
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            if dx*dx + dy*dy > radius*radius:
                continue
            x, y = pixels[:, 0]+dx, pixels[:, 1]+dy
            valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            # source 记录可绘制点的数组下标；target 记录它们要写到的像素编号。
            source = np.flatnonzero(valid)
            target, first = np.unique(y[valid]*width+x[valid], return_index=True)
            source = source[first]  # 同一像素只留首个点；已按深度排序，所以它最近。
            # 不同偏移的圆点也会互相重叠，因此仍需与之前写入的 zbuf 比较。
            nearer = depths[source] < zbuf[target]
            target, source = target[nearer], source[nearer]
            zbuf[target] = depths[source]
            flat[target] = colors[source]
    # 从 inf 变成有限数的像素，就是至少被某个点成功写入过的像素。
    return canvas, int(np.isfinite(zbuf).sum())


def sample_bgr(frame, uv):
    """Bilinear color sampling on the current source frame, preserving BGR."""
    # 根据小数像素位置，从当前视频帧取颜色，返回 N×3 的 BGR 数组。
    # 例如 (10.2, 20.7) 不对应单个整数像素，采用周围四个像素加权混合。
    h, w = frame.shape[:2]
    # 限制坐标在图像范围内，避免取色时数组越界。
    uv = np.clip(uv, [0, 0], [w-1, h-1])
    # x0/y0 是左上角整数坐标，x1/y1 是右下角坐标；边界处可重合。
    x0, y0 = np.floor(uv).astype(int).T
    x1, y1 = np.minimum(x0+1, w-1), np.minimum(y0+1, h-1)
    # ax/ay 是相对于左上角的横向/纵向小数偏移，用来计算四个权重。
    ax, ay = (uv-np.column_stack((x0, y0))).T
    # 左上、右上、左下、右下的权重相加为 1，这就是“双线性插值”。
    color = (frame[y0, x0]*(1-ax)[:, None]*(1-ay)[:, None]
             + frame[y0, x1]*ax[:, None]*(1-ay)[:, None]
             + frame[y1, x0]*(1-ax)[:, None]*ay[:, None]
             + frame[y1, x1]*ax[:, None]*ay[:, None])
    # 混合结果是小数，取最近整数并限制到 0~255，再变回图像使用的 uint8。
    return np.clip(np.rint(color), 0, 255).astype(np.uint8)


def first_person(model, R, center, camera, frame, width, height, active, radius):
    # 生成右上角的第一视角点云图：在“当前相机的位置和朝向”观察已激活点。
    # active 是 N 个布尔值，决定当前时刻哪些点允许显示。
    # 返回 (点云图, 视锥内候选点数, 实际着色像素数)。
    pc = (model.xyz[active]-center) @ R.T  # 批量行向量形式，等价于列向量公式 R@(Pw-C)。
    # 相机只能看前方；剔除 Z<=0 以及过于靠近投影平面的点。
    pc = pc[pc[:, 2] > 1e-8]
    uv = camera.project(pc)
    # 再剔除投影无效、或者落在 ORB-SLAM3 原图像范围之外的点。
    valid = np.isfinite(uv).all(axis=1)
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < camera.width)
    valid &= (uv[:, 1] >= 0) & (uv[:, 1] < camera.height)
    uv, pc = uv[valid], pc[valid]
    fh, fw = frame.shape[:2]
    # uv 的单位基于 ORB-SLAM3 图像尺寸。视频可能被等比例缩放，需要换算后再取色。
    # 第一视角颜色来自当前视频帧，而全局地图颜色来自模型保存的固定点颜色。
    colors = sample_bgr(frame, uv*[fw/camera.width, fh/camera.height])
    # 绘制区域也可能是另一种尺寸，因此再次按输出面板宽高换算像素坐标。
    # 深度缓冲只解决已绘制点之间的遮挡；稀疏点云不能提供完整墙面遮挡关系。
    image, pixels = splat(uv*[width/camera.width, height/camera.height], pc[:, 2],
                          colors, width, height, radius)
    return image, len(pc), pixels


def unit(v):
    # 把方向向量的长度变为 1，只保留方向，便于构造相互垂直的坐标轴。
    n = np.linalg.norm(v)
    # 接近零的向量没有可靠方向，不能通过除以长度来归一化。
    if n < 1e-10:
        raise ValueError("Cannot determine global view orientation")
    return v/n


class GlobalView:
    # 用一个固定的“旁观者相机”观察整个场景，生成下方的全局地图。
    # 它与视频里的真实运动相机不同；固定视角有助于看清点云逐步出现和运动轨迹。
    def __init__(self, model, width, height, azimuth, elevation, fit_percentile=99.5):
        self.width, self.height = width, height
        centers = np.array([p.center for p in model.poses])
        # 1. 确定取景范围。少数异常远点会把视角拉得很远，使主体缩成一小块。
        #    因此只用距中位中心较近的一部分点计算取景范围，但不删除任何原始点。
        median = np.median(model.xyz, axis=0)
        distances = np.linalg.norm(model.xyz-median, axis=1)
        # 99.5 表示以距离的第 99.5 百分位为阈值；100 表示所有点参与取景。
        fitted = distances <= np.percentile(distances, fit_percentile)
        # 将完整相机轨迹也纳入范围，确保取景时不会漏掉轨迹远端。
        bounds = np.vstack((model.xyz[fitted], centers))
        self.fit_points = int(fitted.sum())
        # 用包围盒的中心作为注视目标，再计算覆盖选定范围的半径。
        target = (bounds.min(axis=0)+bounds.max(axis=0))/2
        radius = max(np.linalg.norm(bounds-target, axis=1).max(), 1e-3)
        # 2. 确定场景的“上、前、右”。R 的第二行指向相机下方，取负即向上。
        #    对所有位姿的向上方向求平均，得到统一参考；这不是重力传感器的测量。
        up = unit(np.mean([-p.R[1] for p in model.poses], axis=0))
        forward = model.poses[0].R[2]
        # 去掉前向向量中的上下分量，使其落在与 up 垂直的参考平面上。
        forward = forward-up*np.dot(forward, up)
        if np.linalg.norm(forward) < 1e-6:
            # 如果首帧几乎朝正上/正下看，前向投影接近零，改用首帧右方向。
            forward = model.poses[0].R[0]
            forward -= up*np.dot(forward, up)
        forward = unit(forward)
        # cross 是向量叉乘，用两个方向求出与它们都垂直的第三个方向。
        right = unit(np.cross(forward, up))
        # 3. 用户给的是角度，但 sin/cos 需要弧度，所以先转换。
        # azimuth 是绕场景的方位角；elevation 是相对于参考平面的抬高角度。
        az, el = np.deg2rad([azimuth, elevation])
        # eye_direction 指向“从注视目标到旁观相机”的方向。
        eye_direction = unit((-forward*np.cos(az)+right*np.sin(az))*np.cos(el)+up*np.sin(el))
        # 相机看向目标，所以观察前向与 eye_direction 相反。
        view_forward = -eye_direction
        view_right = unit(np.cross(view_forward, up))
        view_down = np.cross(view_forward, view_right)
        self.R = np.array([view_right, view_down, view_forward])
        # 全局图采用理想透视相机，竖直视场角为 45 度；由画面高度算出像素焦距。
        self.f = .5*height/math.tan(math.radians(45)/2)
        relative = (bounds-target) @ self.R.T
        # 4. 求旁观相机应后退多远，才能同时装下选定点云范围和整条轨迹。
        # 投影横坐标约为 f*x/(z+distance)。要求它不超过画面半宽，就能反求距离。
        # .43*width 与 .38*height 比半宽/半高稍小，给画面边缘及文字留一些空间。
        # 三项分别约束水平范围、竖直范围、以及所有参与取景的点位于相机前方。
        distance = max(np.max(np.abs(relative[:, 0])*self.f/(.43*width)-relative[:, 2]),
                       np.max(np.abs(relative[:, 1])*self.f/(.38*height)-relative[:, 2]),
                       -relative[:, 2].min()+.05*radius)
        self.eye = target+eye_direction*distance
        # 相机视锥小图标的大小按场景半径设置，与真实相机的物理大小无关。
        self.frustum_size = .035*radius
        # 点云和旁观视角都固定，可以一次投影全部点，后续只筛选哪些点显示。
        # 这里仍投影全部原始点；参与取景范围计算以外的点可能落到画面之外。
        self.uv, self.z = self.project(model.xyz)

    def project(self, xyz):
        # 世界坐标 -> 旁观相机坐标 -> 全局面板像素坐标。
        # 返回 (N×2 像素坐标, N 个深度)，供画点、画轨迹和画相机共用。
        pc = (np.asarray(xyz)-self.eye) @ self.R.T
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = pc[:, :2]/pc[:, 2:3]*self.f+[self.width/2, self.height/2]
        return uv, pc[:, 2]

    def map_image(self, model, active, radius):
        # 用初始化时缓存的投影坐标，绘制当前已激活的地图点。
        # 全局图使用模型固定颜色；splat 返回图像和像素数，这里只取图像 [0]。
        return splat(self.uv[active], self.z[active], model.bgr[active],
                     self.width, self.height, radius)[0]

    def draw_motion(self, image, centers, current, R, camera):
        # 在传入的 image 上直接叠加轨迹和相机图标，调用后原图像会发生变化。
        # centers 是截至当前的已注册位置，current 是当前帧可能经过插值的位置。
        path = np.vstack((centers, current)) if len(centers) else current[None]
        uv, z = self.project(path)
        if np.isfinite(uv).all() and (z > 0).all():
            # False 表示轨迹不首尾闭合；LINE_AA 用抗锯齿使线条边缘更平滑。
            cv2.polylines(image, [np.rint(uv).astype(np.int32)], False,
                          (25, 225, 250), 2, cv2.LINE_AA)
        # 用图像四角构造相机视锥：像从镜头向前张开的四棱锥，表示相机看向哪里。
        # 四角是有畸变图像上的位置，需要先近似反求对应的无畸变相机射线。
        corners = np.array([[0, 0], [camera.width, 0],
                            [camera.width, camera.height], [0, camera.height]], float)
        # 先忽略畸变，按焦距/主点估计射线，然后通过投影误差迭代修正 12 次。
        # 这是供图标绘制使用的数值近似，不保证任意强畸变模型都能收敛。
        p = camera.params
        fxy = [p[0], p[0]] if camera.model.startswith("SIMPLE") or camera.model == "RADIAL" else p[:2]
        principal = p[1:3] if camera.model.startswith("SIMPLE") or camera.model == "RADIAL" else p[2:4]
        rays_xy = (corners-principal)/fxy
        for _ in range(12):
            # 将估计射线投影回图像，与目标四角比较，再用误差修正估计。
            rays_xy += (corners-camera.project(np.column_stack((rays_xy, np.ones(4)))))/fxy
        rays = np.column_stack((rays_xy, np.ones(4)))*self.frustum_size
        # rays 在当前运动相机坐标中；行向量乘 R 反变换到世界方向，再加相机位置。
        # vertices 共 5 个顶点：相机中心，以及向前展开的四个角。
        vertices = np.vstack((current, current+rays @ R))
        uv, z = self.project(vertices)
        if (z > 0).all() and np.isfinite(uv).all():
            uv = np.rint(uv).astype(int)
            # 前 4 条边从相机中心连向四角，后 4 条边连接四角构成矩形框。
            for a, b in [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]:
                cv2.line(image, tuple(uv[a]), tuple(uv[b]), (70, 250, 125), 2, cv2.LINE_AA)
            cv2.circle(image, tuple(uv[0]), 3, (70, 250, 125), -1, cv2.LINE_AA)


def label(image, text, xy=(12, 22), scale=.43):
    # 在图像上绘制带深色底框的文字，避免文字与点云/视频背景混在一起。
    # xy 是文字基线左端的像素位置，scale 是字号比例；函数直接修改 image。
    # 默认 Hershey 字体用于当前英文标签，不直接支持普通中文字符。
    size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = xy
    # 先量文字尺寸，底框四周多留几像素；rectangle 的 -1 表示填满矩形。
    cv2.rectangle(image, (x-5, y-size[1]-5), (x+size[0]+5, y+baseline+3), (15, 17, 14), -1)
    cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (235, 239, 235), 1, cv2.LINE_AA)
