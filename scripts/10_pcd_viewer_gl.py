#!/usr/bin/env python3
"""OpenGL 点云查看器 —— 从「画三角形」到「看 20 万点的建筑」

⚠️ 必须用【系统 python3】：
    conda 自带的老 libstdc++ 不支持 GLIBCXX_3.4.30 → mesa 驱动加载失败
    → OpenGL 完全不可用。这是本机最容易踩的坑。

运行：
    /usr/bin/python3 scripts/10_pcd_viewer_gl.py map.pcd
    /usr/bin/python3 scripts/10_pcd_viewer_gl.py map.pcd traj.txt

    或先装好 launcher（会自动挑对解释器）：
    bash scripts/install_pcd_gl.sh && pcd-gl map.pcd


和教程里的三角形比，多了三样东西
────────────────────────────────
    ① 顶点从 3 个变成几十万个，且数据从文件读（PCD 解析）
    ② 顶点着色器里多了一个 4×4 矩阵（MVP）—— 这才是能旋转缩放的根源
    ③ 鼠标交互：拖动改相机参数，每帧重新上传矩阵


操作
────
    左键拖动      旋转（绕目标点转）
    滚轮          缩放（拉近拉远）
    右键 / 中键拖动  平移（移动关注的焦点）
    R             复位视角
    Esc           退出


为什么 OpenGL 比浏览器快
────────────────────────
    浏览器（plotly）要走 WebGL + JS 胶水层，60 万点就开始掉帧。
    这里点云**一次性躺在显存里**，每帧只改一个 64 字节的矩阵，
    GPU 直接照着画 —— 100 万点也很轻松。
"""
from __future__ import annotations

import ctypes
import re
import sys
import time
from pathlib import Path

import numpy as np
from OpenGL import GL

try:
    import glfw
except ImportError:
    sys.exit(
        "❌ 缺 glfw。装一下：\n"
        "   /usr/bin/python3 -m pip install --user PyOpenGL glfw"
    )

# ═══════════════════════════════════════════════════════════
#  GLSL
# ═══════════════════════════════════════════════════════════
VERTEX_SHADER = """#version 330 core
layout(location = 0) in vec3 aPos;      // 点云坐标（米）
layout(location = 1) in vec3 aColor;    // 预计算好的颜色

uniform mat4 uMVP;                       // ⭐ 模型-视图-投影矩阵：唯一每帧变化的东西

out vec3 vColor;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);   // 顶点从"世界坐标"变到"裁剪空间"
    vColor = aColor;                         // 直接透传
}
"""

FRAGMENT_SHADER = """#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
"""


# ═══════════════════════════════════════════════════════════
#  4×4 矩阵（教学用，手写。实际项目常用 glm / eigen）
#  ⚠️ OpenGL 的矩阵是【列主序】，所以给 GL 之前要转置
# ═══════════════════════════════════════════════════════════
def perspective(fovy_deg: float, aspect: float, near: float, far: float) -> np.ndarray:
    """透视投影：近大远小。把视锥体压成 [-1,1]³ 的裁剪空间。"""
    f = 1.0 / np.tan(np.radians(fovy_deg) / 2.0)
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (far + near) / (near - far)
    m[2, 3] = (2 * far * near) / (near - far)
    m[3, 2] = -1.0
    return m


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """视图矩阵：把世界坐标变到「相机在原点、朝 -z 看」的坐标系。"""
    f = target - eye
    f = f / (np.linalg.norm(f) + 1e-12)          # 前方向
    s = np.cross(f, up)
    s = s / (np.linalg.norm(s) + 1e-12)          # 右方向
    u = np.cross(s, f)                            # 上方向

    m = np.eye(4, dtype=np.float32)
    m[0, :3] = s
    m[1, :3] = u
    m[2, :3] = -f
    m[0, 3] = -np.dot(s, eye)
    m[1, 3] = -np.dot(u, eye)
    m[2, 3] = np.dot(f, eye)
    return m


# ═══════════════════════════════════════════════════════════
#  PCD 读取（支持 binary 和 ascii）
# ═══════════════════════════════════════════════════════════
def load_pcd(path: Path) -> np.ndarray:
    """读 PCD → (N,4) float32 [x, y, z, intensity]。"""
    data = path.read_bytes()
    m = re.search(rb"DATA\s+(\w+)\s*\n", data)
    if not m:
        raise ValueError("不是 PCD（找不到 DATA 行）")
    mode = m.group(1).decode().lower()
    head = data[: m.end()].decode("ascii", "replace")

    def field(name: str) -> str:
        mm = re.search(rf"^{name}\s+(.+)$", head, re.M | re.I)
        return mm.group(1).strip() if mm else ""

    fields = field("FIELDS").split()
    sizes = [int(x) for x in field("SIZE").split()]
    types = field("TYPE").split()
    counts = [int(x) for x in field("COUNT").split()] or [1] * len(fields)
    npts = int(field("POINTS") or 0)

    if not {"x", "y", "z"} <= set(fields):
        raise ValueError("点云里没有 x/y/z")
    i_int = fields.index("intensity") if "intensity" in fields else (
        fields.index("reflectivity") if "reflectivity" in fields else None)

    if mode == "ascii":
        body = data[m.end():].decode("ascii", "replace")
        rows = [ln.split() for ln in body.splitlines() if ln.strip() and not ln.startswith("#")]
        arr = np.array([[float(v) for v in r[: len(fields)]] for r in rows], dtype=np.float32)
        out = np.zeros((arr.shape[0], 4), dtype=np.float32)
        out[:, 0] = arr[:, fields.index("x")]
        out[:, 1] = arr[:, fields.index("y")]
        out[:, 2] = arr[:, fields.index("z")]
        if i_int is not None:
            out[:, 3] = arr[:, i_int]
        return out

    if mode != "binary":
        raise ValueError(f"不支持的 DATA {mode}")

    # binary：算每个字段的偏移
    offsets, step = [], 0
    for sz, cnt in zip(sizes, counts):
        offsets.append(step)
        step += sz * cnt

    n_avail = (len(data) - m.end()) // step
    n = min(npts, n_avail) if npts else n_avail
    raw = np.frombuffer(data, dtype=np.uint8, count=n * step, offset=m.end()).reshape(n, step)

    def read(idx: int) -> np.ndarray:
        off, sz, tp = offsets[idx], sizes[idx], types[idx].upper()
        chunk = raw[:, off:off + sz]
        if tp == "F":
            return chunk.view(np.float32 if sz == 4 else np.float64).ravel().astype(np.float32)
        if tp == "I":
            return chunk.view({1: np.int8, 2: np.int16, 4: np.int32}[sz]).ravel().astype(np.float32)
        return chunk.view({1: np.uint8, 2: np.uint16, 4: np.uint32}[sz]).ravel().astype(np.float32)

    out = np.zeros((n, 4), dtype=np.float32)
    out[:, 0] = read(fields.index("x"))
    out[:, 1] = read(fields.index("y"))
    out[:, 2] = read(fields.index("z"))
    if i_int is not None:
        out[:, 3] = read(i_int)
    return out


def load_traj(path: Path) -> np.ndarray:
    """读 TUM 轨迹 → (N,3)。"""
    rows = []
    for ln in path.read_text().splitlines():
        s = ln.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split()
        if len(p) >= 4:
            try:
                rows.append((float(p[1]), float(p[2]), float(p[3])))
            except ValueError:
                continue
    return np.array(rows, dtype=np.float32) if rows else np.zeros((0, 3), np.float32)


def frame_bounds(pts: np.ndarray):
    """用 2%~98% 分位数定视野，自动忽略远处离群点。

    MID-360 量程 40 m，但玻璃/镜面会反射出上百米的假点。
    直接 min/max 会把相机拉到老远，主体小得看不见。
    """
    lo = np.percentile(pts[:, :3], 2, axis=0)
    hi = np.percentile(pts[:, :3], 98, axis=0)
    center = ((lo + hi) / 2).astype(np.float64)
    radius = float(np.linalg.norm(hi - lo)) / 2
    return center, max(radius, 0.5), lo, hi


def height_colors(xyz: np.ndarray) -> np.ndarray:
    """按高度映射到 viridis 风格的颜色（在 CPU 上算好，塞进顶点属性）。"""
    z = xyz[:, 2]
    lo, hi = float(z.min()), float(z.max())
    t = (z - lo) / max(hi - lo, 1e-9)

    # 简化版 viridis：深紫 → 蓝 → 青 → 绿 → 黄
    stops = np.array([
        [0.267, 0.005, 0.329],
        [0.188, 0.407, 0.554],
        [0.128, 0.567, 0.551],
        [0.369, 0.789, 0.383],
        [0.993, 0.906, 0.144],
    ], dtype=np.float32)
    pos = t * (len(stops) - 1)
    i0 = np.clip(np.floor(pos).astype(int), 0, len(stops) - 2)
    frac = (pos - i0)[:, None]
    return stops[i0] * (1 - frac) + stops[i0 + 1] * frac


# ═══════════════════════════════════════════════════════════
#  轨道相机
# ═══════════════════════════════════════════════════════════
class OrbitCamera:
    def __init__(self, center: np.ndarray, radius: float):
        self.center = center.astype(np.float64)   # 看哪儿
        self.dist = radius * 1.7                  # 离多远
        self.theta = np.radians(45.0)             # 水平角
        self.phi = np.radians(38.0)               # 俯仰角（0=水平，90=俯视）
        self.fov = 50.0

    def eye(self) -> np.ndarray:
        """相机位置 = 目标点 + 球坐标偏移。"""
        cp, sp = np.cos(self.phi), np.sin(self.phi)
        return self.center + self.dist * np.array([
            np.cos(self.theta) * cp,
            np.sin(self.theta) * cp,
            sp,
        ], dtype=np.float64)

    def mvp(self, aspect: float) -> np.ndarray:
        proj = perspective(self.fov, aspect, 0.05, max(self.dist * 50, 1000))
        view = look_at(self.eye(), self.center, np.array([0.0, 0.0, 1.0]))
        return (proj @ view).astype(np.float32)

    def orbit(self, dx: float, dy: float) -> None:
        self.theta += dx * 0.008
        self.phi = float(np.clip(self.phi + dy * 0.008, np.radians(-89), np.radians(89)))

    def zoom(self, amount: float) -> None:
        self.dist = float(np.clip(self.dist * (1.0 - amount * 0.12), 0.2, 1e6))

    def pan(self, dx: float, dy: float) -> None:
        eye = self.eye()
        fwd = self.center - eye
        fwd /= np.linalg.norm(fwd) + 1e-12
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right) + 1e-12
        up = np.cross(right, fwd)
        scale = self.dist * 0.0016
        self.center += (-right * dx + up * dy) * scale


# ═══════════════════════════════════════════════════════════
#  主程序
# ═══════════════════════════════════════════════════════════
def compile_shader(src: str, kind) -> int:
    sh = GL.glCreateShader(kind)
    GL.glShaderSource(sh, src)
    GL.glCompileShader(sh)
    if not GL.glGetShaderiv(sh, GL.GL_COMPILE_STATUS):
        raise RuntimeError(GL.glGetShaderInfoLog(sh).decode())
    return sh


def make_program(vs: str, fs: str) -> int:
    p = GL.glCreateProgram()
    a, b = compile_shader(vs, GL.GL_VERTEX_SHADER), compile_shader(fs, GL.GL_FRAGMENT_SHADER)
    GL.glAttachShader(p, a)
    GL.glAttachShader(p, b)
    GL.glLinkProgram(p)
    if not GL.glGetProgramiv(p, GL.GL_LINK_STATUS):
        raise RuntimeError(GL.glGetProgramInfoLog(p).decode())
    GL.glDeleteShader(a)
    GL.glDeleteShader(b)
    return p


def upload_points(pts: np.ndarray, fixed_color=None) -> tuple[int, int, int]:
    """把点云搬进显存。返回 (vao, vbo, 点数)。

    fixed_color 给了就用它（轨迹这类需要醒目颜色的），
    否则按高度着色。
    """
    xyz = pts[:, :3].astype(np.float32)
    if fixed_color is not None:
        rgb = np.tile(np.asarray(fixed_color, dtype=np.float32), (len(xyz), 1))
    else:
        rgb = height_colors(xyz)
    # 交错布局：[x y z r g b] × N —— 一次上传，GPU 连续读
    verts = np.hstack([xyz, rgb]).astype(np.float32)

    vao = GL.glGenVertexArrays(1)
    GL.glBindVertexArray(vao)
    vbo = GL.glGenBuffers(1)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
    GL.glBufferData(GL.GL_ARRAY_BUFFER, verts.nbytes, verts, GL.GL_STATIC_DRAW)

    stride = 6 * 4
    GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(0))
    GL.glEnableVertexAttribArray(0)
    GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(12))
    GL.glEnableVertexAttribArray(1)
    GL.glBindVertexArray(0)
    return vao, vbo, verts.shape[0]


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if not args:
        print("用法: /usr/bin/python3 scripts/10_pcd_viewer_gl.py <map.pcd> [traj.txt]")
        return 1

    pcd_path = Path(args[0]).expanduser()
    traj_path = Path(args[1]).expanduser() if len(args) > 1 else None
    if not pcd_path.is_file():
        sys.exit(f"❌ 文件不存在: {pcd_path}")

    t0 = time.time()
    pts = load_pcd(pcd_path)
    traj = load_traj(traj_path) if traj_path and traj_path.is_file() else None
    print(f"读入 {len(pts):,} 点  ({time.time()-t0:.2f}s)  {pcd_path.name}")
    if traj is not None:
        print(f"轨迹 {len(traj):,} 位姿  {traj_path.name}")

    center, radius, lo, hi = frame_bounds(pts)
    print(f"视野: x {lo[0]:.1f}~{hi[0]:.1f}  y {lo[1]:.1f}~{hi[1]:.1f}  "
          f"z {lo[2]:.1f}~{hi[2]:.1f}  (半径 {radius:.1f} m)")

    if not glfw.init():
        sys.exit("❌ glfw.init() 失败")
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.SAMPLES, 4)
    win = glfw.create_window(1280, 820, f"OpenGL 点云查看器 — {pcd_path.name}", None, None)
    if not win:
        glfw.terminate()
        sys.exit("❌ 创建窗口失败")
    glfw.make_context_current(win)
    glfw.swap_interval(1)

    print(f"OpenGL {GL.glGetString(GL.GL_VERSION).decode()} | "
          f"{GL.glGetString(GL.GL_RENDERER).decode()}")

    program = make_program(VERTEX_SHADER, FRAGMENT_SHADER)
    u_mvp = GL.glGetUniformLocation(program, "uMVP")

    vao_pts, vbo_pts, n_pts = upload_points(pts)
    vao_traj = vbo_traj = n_traj = 0
    traj_cam = None          # 按 T 时切到这个视角
    if traj is not None and len(traj) > 1:
        # ⚠️ 轨迹必须用固定醒目的颜色。
        #    早先这里是按高度着色的，而轨迹 z 被填成常数 0，
        #    结果全染成 viridis 最暗那档（深紫），在深色背景上根本看不见。
        vao_traj, vbo_traj, n_traj = upload_points(
            np.hstack([traj, np.zeros((len(traj), 1), np.float32)]),
            fixed_color=(1.0, 0.18, 0.18))
        t_center, t_radius, t_lo, t_hi = frame_bounds(traj)
        traj_cam = OrbitCamera(t_center, max(t_radius, 1.5))

    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
    GL.glEnable(GL.GL_MULTISAMPLE)

    cam = OrbitCamera(center, radius)

    # ── 鼠标状态 ──
    state = {"left": False, "right": False, "x": 0.0, "y": 0.0}

    def on_button(w, button, action, mods):
        if button == glfw.MOUSE_BUTTON_LEFT:
            state["left"] = action == glfw.PRESS
        elif button in (glfw.MOUSE_BUTTON_RIGHT, glfw.MOUSE_BUTTON_MIDDLE):
            state["right"] = action == glfw.PRESS

    def on_cursor(w, x, y):
        dx, dy = x - state["x"], y - state["y"]
        state["x"], state["y"] = x, y
        if state["left"]:
            cam.orbit(dx, dy)
        elif state["right"]:
            cam.pan(dx, dy)

    def on_scroll(w, dx, dy):
        cam.zoom(dy)

    glfw.set_mouse_button_callback(win, on_button)
    glfw.set_cursor_pos_callback(win, on_cursor)
    glfw.set_scroll_callback(win, on_scroll)

    keys = "  左键拖动=旋转  滚轮=缩放  右键拖动=平移  R=全局视角"
    if n_traj > 1:
        keys += "  T=锁定轨迹"
    print("\n" + keys + "  Esc=退出")

    while not glfw.window_should_close(win):
        if glfw.get_key(win, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(win, True)
        if glfw.get_key(win, glfw.KEY_R) == glfw.PRESS:
            cam = OrbitCamera(center, radius)
        if glfw.get_key(win, glfw.KEY_T) == glfw.PRESS and traj_cam is not None:
            cam = OrbitCamera(traj_cam.center, traj_cam.dist / 1.7)

        w, h = glfw.get_framebuffer_size(win)
        GL.glViewport(0, 0, w, h)
        GL.glClearColor(0.055, 0.055, 0.075, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        mvp = cam.mvp(w / max(h, 1))
        GL.glUseProgram(program)
        # ⚠️ OpenGL 期望列主序；numpy 的 @ 结果是行主序表示，所以要 .T
        GL.glUniformMatrix4fv(u_mvp, 1, GL.GL_TRUE, mvp.astype(np.float32))

        GL.glPointSize(1.6)
        GL.glBindVertexArray(vao_pts)
        GL.glDrawArrays(GL.GL_POINTS, 0, n_pts)

        if n_traj > 1:
            GL.glBindVertexArray(vao_traj)
            # 先用粗线连起来（路径形状），再叠一层大点（每个位姿）
            GL.glLineWidth(3.0)
            GL.glDrawArrays(GL.GL_LINE_STRIP, 0, n_traj)
            GL.glPointSize(7.0)
            GL.glDrawArrays(GL.GL_POINTS, 0, n_traj)

        glfw.swap_buffers(win)
        glfw.poll_events()

    glfw.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
