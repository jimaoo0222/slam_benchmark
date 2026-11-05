"""
绘图环境统一配置
================

路径: slam_benchmark/common/plot_setup.py

用法（在脚本最顶部，必须早于 import matplotlib）:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from common import plot_setup          # 会自动完成所有修复与配置
    import matplotlib.pyplot as plt        # 之后再导入 matplotlib
    from mpl_toolkits.mplot3d import Axes3D

    plot_setup.save(fig, "result.png")     # 统一的保存方式

本模块解决 Ubuntu + pip --user 环境下的三个坑（详见下方注释）。
"""

import os
import sys

# =====================================================================
# 坑 1：mpl_toolkits 被系统的 .pth 预缓存，导致 3D 绘图失效
# =====================================================================
# 症状:
#     ValueError: Unknown projection '3d'
#     ImportError: cannot import name 'docstring' from 'matplotlib'
#
# 原因:
#     /usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth 会在解释器
#     【启动时】就把 mpl_toolkits 写进 sys.modules，并指向系统的旧版
#     matplotlib 3.5（因为它是带 __init__.py 的普通包，会赢过 pip 装在
#     ~/.local 里的 namespace 包版本）。于是新版 matplotlib 3.10 配合旧版
#     mpl_toolkits 工作 → 调用了 3.10 里已删除的 API → 报错。
#
# 修法（两处，缺一不可）:
#     (a) 让 ~/.local 的 mpl_toolkits 也成为普通包（本模块的 fix_mpl_toolkits
#         会在必要时自动创建 __init__.py）—— 这样它在 sys.path 中更靠前，
#         能在路径扫描时胜出；
#     (b) 清掉 sys.modules 里被 .pth 预先塞进去的缓存（在下面执行）。
#
# 注意:
#     不要 apt 卸载系统 matplotlib！ros-humble-rqt-plot 依赖它。


def fix_mpl_toolkits(verbose: bool = False) -> None:
    """修复 mpl_toolkits 版本冲突。幂等，可重复调用。"""
    # --- (a) 确保用户目录的 mpl_toolkits 是普通包 ---
    for p in sys.path:
        if ".local" not in p or "site-packages" not in p:
            continue
        mt = os.path.join(p, "mpl_toolkits")
        if os.path.isdir(mt):
            init = os.path.join(mt, "__init__.py")
            if not os.path.exists(init):
                try:
                    open(init, "a").close()
                    if verbose:
                        print(f"[plot_setup] 已创建 {init}")
                except OSError as e:
                    print(f"[plot_setup] 警告：无法创建 {init}: {e}")

    # --- (b) 清掉 .pth 预缓存的模块 ---
    for mod in [m for m in list(sys.modules)
                if m == "mpl_toolkits" or m.startswith("mpl_toolkits.")]:
        del sys.modules[mod]


fix_mpl_toolkits()


# =====================================================================
# 坑 2：中文字体 —— 不配置的话所有中文都是方框 □□□
# =====================================================================
# matplotlib 默认字体 DejaVu Sans 不含汉字。
# 用「字体列表 + 回退」逐个尝试，直到能渲染为止。
#
# 查看系统有哪些中文字体:
#     fc-list :lang=zh

import matplotlib  # noqa: E402

# 坑 3：无显示器环境（SSH / 服务器）必须用 Agg 后端，否则保存图片会报错。
#      需要交互式窗口时改成 "TkAgg"。
if not os.environ.get("DISPLAY") or os.environ.get("SLAM_BENCH_HEADLESS") == "1":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

CJK_FONTS = [
    "Noto Sans CJK JP",      # 系统 noto-cjk 质量最好（matplotlib 只注册了 JP 面）
    "Noto Sans CJK SC",      # 若哪天注册上了 SC 面会优先用
    "Droid Sans Fallback",   # 兜底，覆盖全部 CJK
    "WenQuanYi Micro Hei",
    "AR PL UMing CN",
    "DejaVu Sans",           # 最后退回默认（至少英文数字正常）
]

plt.rcParams["font.sans-serif"] = CJK_FONTS
plt.rcParams["font.family"] = "sans-serif"
# 汉字字体常缺 U+2212(数学减号)，改用 ASCII 的 '-'，否则负号也是方框
plt.rcParams["axes.unicode_minus"] = False

# 统一图表风格
plt.rcParams["figure.dpi"] = 110
plt.rcParams["savefig.bbox"] = "tight"
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3


# =====================================================================
# 便捷函数
# =====================================================================

def save(fig, filename: str, outdir: str | None = None) -> str:
    """统一保存图片，返回完整路径。缺省保存到调用脚本所在目录。"""
    if outdir is None:
        outdir = os.path.dirname(os.path.abspath(sys.argv[0])) or "."
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, filename)
    fig.savefig(path)
    plt.close(fig)
    print(f"✅ 已保存: {path}")
    return path


def check(verbose: bool = True) -> bool:
    """自检：打印 matplotlib 与 3D 是否可用。"""
    ok = True
    if verbose:
        print(f"[plot_setup] matplotlib {matplotlib.__version__}"
              f" | 后端 {matplotlib.get_backend()}")
    try:
        import mpl_toolkits
        if "dist-packages" in mpl_toolkits.__path__[0]:
            ok = False
            if verbose:
                print("[plot_setup] ⚠️ mpl_toolkits 仍指向系统目录，"
                      "3D 绘图可能失败")
        else:
            if verbose:
                print(f"[plot_setup] ✅ mpl_toolkits → "
                      f"{mpl_toolkits.__path__[0]}")
    except Exception as e:
        ok = False
        if verbose:
            print(f"[plot_setup] ❌ 导入 mpl_toolkits 失败: {e}")

    try:
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        if verbose:
            print("[plot_setup] ✅ mplot3d 可用")
    except Exception as e:
        ok = False
        if verbose:
            print(f"[plot_setup] ❌ mplot3d 不可用: {e}")
    return ok


if __name__ == "__main__":
    print("=" * 60)
    print("绘图环境自检")
    print("=" * 60)
    check()
