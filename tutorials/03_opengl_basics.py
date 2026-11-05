#!/usr/bin/env python3
"""OpenGL 第一步：画一个彩色三角形

⚠️ 必须用【系统 python3】跑，不能用 conda 的：
    conda 自带的老版本 libstdc++ 不支持 GLIBCXX_3.4.30，
    导致 mesa 驱动（radeonsi_dri.so）加载失败 → OpenGL 完全不可用。

正确运行方式：
    /usr/bin/python3 tutorials/03_opengl_basics.py
    # 或
    env -u LD_LIBRARY_PATH /usr/bin/python3 tutorials/03_opengl_basics.py


一、OpenGL 是什么
────────────────
它不是"画图函数库"，而是一台**状态机**：
你不停告诉它「现在用这个 shader」「现在绑这个缓冲」，最后喊 glDrawArrays()
才真正画。真正的计算全在 GPU 上，CPU 只负责发指令。


二、渲染管线（现代 OpenGL 3.3+ 必走）
────────────────────────────────────
    CPU 顶点数据 ──► ① VBO（显存里的字节块）
                          │
                  ② VAO 记下「怎么读这块内存」
                          │
                  ③ 顶点着色器：算每个顶点在屏幕上的位置
                          │
                  ④ 光栅化：把三角形内部填成一个个像素
                          │
                  ⑤ 片段着色器：决定每个像素的颜色
                          │
                  ⑥ 写进帧缓冲 → 显示


三、4 个必记的名字
─────────────────
    VBO   Vertex Buffer Object   显存里的字节块，顶点数据住这里
    VAO   Vertex Array Object    "读取说明书"，画之前 bind 一下
    Shader Program               GPU 上跑的两段小程序（GLSL 写）
    glDrawArrays()               真正触发绘制的那一句

关键：GPU 不认识"三角形""点云"，它只认识
     「从 VBO 的哪个位置、按什么格式、读多少个 float」——
     这些全由 VAO + glVertexAttribPointer 描述。


四、从这里到"点云查看器"只差两步
────────────────────────────────
    1. glDrawArrays(GL_POINTS, 0, N)   ← 三角形换成点
    2. 顶点着色器里加个矩阵            ← 让点云能旋转缩放
"""
import ctypes
import sys

import numpy as np
from OpenGL import GL

try:
    import glfw
except ImportError:
    sys.exit("❌ 缺 glfw。装一下：\n   /usr/bin/python3 -m pip install --user glfw PyOpenGL")

# ─────────────────────────────────────────────────────────────
#  GLSL：跑在 GPU 上的两段程序
#    顶点着色器 —— 每个顶点执行一次，决定它落在屏幕哪里
#    片段着色器 —— 每个像素执行一次，决定它什么颜色
# ─────────────────────────────────────────────────────────────
VERTEX_SHADER = """#version 330 core
layout(location = 0) in vec3 aPos;      // 属性 0：位置
layout(location = 1) in vec3 aColor;    // 属性 1：颜色

out vec3 vColor;                         // 传给片段着色器（会自动插值）

void main() {
    gl_Position = vec4(aPos, 1.0);       // 裁剪空间：xy ∈ [-1,1] 正好是整窗口
    vColor = aColor;
}
"""

FRAGMENT_SHADER = """#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
"""

# 3 个顶点，每个 6 个 float：x y z  r g b
TRIANGLE = np.array([
     0.0,  0.7, 0.0,   1.00, 0.25, 0.25,   # 上：红
    -0.7, -0.6, 0.0,   0.25, 1.00, 0.45,   # 左下：绿
     0.7, -0.6, 0.0,   0.35, 0.55, 1.00,   # 右下：蓝
], dtype=np.float32)


def compile_shader(src: str, kind) -> int:
    """把 GLSL 源码编译成 GPU 能执行的对象。"""
    sh = GL.glCreateShader(kind)
    GL.glShaderSource(sh, src)
    GL.glCompileShader(sh)
    if not GL.glGetShaderiv(sh, GL.GL_COMPILE_STATUS):
        raise RuntimeError("shader 编译失败:\n" + GL.glGetShaderInfoLog(sh).decode())
    return sh


def make_program(vs_src: str, fs_src: str) -> int:
    """把顶点着色器 + 片段着色器链接成一个可用的 program。"""
    vs = compile_shader(vs_src, GL.GL_VERTEX_SHADER)
    fs = compile_shader(fs_src, GL.GL_FRAGMENT_SHADER)
    prog = GL.glCreateProgram()
    GL.glAttachShader(prog, vs)
    GL.glAttachShader(prog, fs)
    GL.glLinkProgram(prog)
    if not GL.glGetProgramiv(prog, GL.GL_LINK_STATUS):
        raise RuntimeError("链接失败:\n" + GL.glGetProgramInfoLog(prog).decode())
    GL.glDeleteShader(vs)      # 链接完就没用了，GPU 已拿到编译好的版本
    GL.glDeleteShader(fs)
    return prog


def main() -> int:
    if not glfw.init():
        sys.exit("❌ glfw.init() 失败（可能是没有图形环境）")

    # 告诉 GLFW：我要 3.3 core profile（不指定的话可能给兼容模式，用不了 #version 330）
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    win = glfw.create_window(900, 650, "OpenGL 第一步：彩色三角形（Esc 退出）", None, None)
    if not win:
        glfw.terminate()
        sys.exit("❌ 创建窗口失败")

    glfw.make_context_current(win)
    glfw.swap_interval(1)          # 垂直同步，避免风扇狂转

    # ── 打个招呼，确认 GPU 栈是好的 ──
    print("OpenGL  :", GL.glGetString(GL.GL_VERSION).decode())
    print("渲染器  :", GL.glGetString(GL.GL_RENDERER).decode())
    print("GLSL    :", GL.glGetString(GL.GL_SHADING_LANGUAGE_VERSION).decode())
    print("按 Esc 或关窗口退出")

    # ═══════════════════════════════════════════════════════
    #  初始化（只做一次）
    # ═══════════════════════════════════════════════════════
    program = make_program(VERTEX_SHADER, FRAGMENT_SHADER)

    # ① VAO：记录"从 VBO 怎么读顶点"
    vao = GL.glGenVertexArrays(1)
    GL.glBindVertexArray(vao)

    # ② VBO：把顶点数据搬进显存
    vbo = GL.glGenBuffers(1)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
    GL.glBufferData(GL.GL_ARRAY_BUFFER, TRIANGLE.nbytes, TRIANGLE, GL.GL_STATIC_DRAW)

    # ③ 描述数据布局：每个顶点 24 字节 = 6 个 float
    stride = 6 * 4
    GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(0))
    GL.glEnableVertexAttribArray(0)                    # 位置：偏移 0，3 个 float
    GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(12))
    GL.glEnableVertexAttribArray(1)                    # 颜色：偏移 12 字节，3 个 float

    GL.glBindVertexArray(0)

    # ═══════════════════════════════════════════════════════
    #  主循环
    # ═══════════════════════════════════════════════════════
    while not glfw.window_should_close(win):
        if glfw.get_key(win, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(win, True)

        # ① 清屏
        GL.glClearColor(0.08, 0.08, 0.11, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        # ② 选 shader、选 VAO
        GL.glUseProgram(program)
        GL.glBindVertexArray(vao)

        # ③ ⭐ 开画
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

        glfw.swap_buffers(win)     # 把画好的内容显示出来（双缓冲）
        glfw.poll_events()

    glfw.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
