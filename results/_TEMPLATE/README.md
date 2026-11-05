# 实验目录模板

**采数据前**：复制一份，改名成当天日期 + 场地。

```bash
cd ~/slam_benchmark
cp -r results/_TEMPLATE results/2026-10-10_场地A
cd results/2026-10-10_场地A
```

---

## 流程（注意：顺序别颠倒）

| # | 做什么 | 产出 |
|---|---|---|
| 1 | **录 3~5 个包** —— 每遍都：从原点出发 → 走固定路线 → 每点停 5 秒 → 回到原点 | `run1/ run2/ run3/` |
| 2 | **量距离** —— 卷尺量相邻停车点的距离（各段拉开：1/3/5 m），**量一次就行** | `distances.txt` |
| 3 | **每个包都跑一遍算法** —— GLIM 自动写轨迹；其他算法用 `traj_recorder.py` | `bag*_glim.txt` … |
| 4 | **跑评测** —— 四项指标 | `protocol_report.png` |
| 5 | **填表** | `对比表.md` |

> 注意：**为什么要录多个包**：
> 只录一个，结论可能是「运气好」；**多个包才能说「普遍稳定」** ——
> 这就是「② 可重复性」的数据来源。
>
> **一次采完所有包就行**，不用每个算法重采一遍。

---

## 目录里该有什么

```
2026-10-10_场地A/
├── run1/  run2/  run3/      ← 多个包（同一路径走的 3~5 遍）
├── distances.txt            ← 卷尺量的相邻停车点距离（不用量坐标）
├── bag1_glim.txt            ← 各算法在每个包上的轨迹
├── bag2_glim.txt
├── bag3_glim.txt
├── protocol_report.png      ← 自动生成
└── 对比表.md                 ← 最终结果
```

`gt.txt` 是**可选的**（只在用老工具 `benchmark_eval.py --gt` 时才需要）。

---

## 常用命令

```bash
# 主线：四项指标（不需要绝对真值）
python3 ../../scripts/protocol_eval.py bag1_glim.txt --distances 1.02,3.01,5.03

# ② 可重复性：把多个包的轨迹一起传进去
python3 ../../scripts/protocol_eval.py bag1_glim.txt bag2_glim.txt bag3_glim.txt

# （可选）绝对坐标评测，需要 gt.txt
python3 ../../scripts/benchmark_eval.py --gt gt.txt --est bag1_glim.txt
```

---

## 为什么要「录多个包 + 离线回放」

```
同一路径录多个包  →  每个包都让所有算法回放一遍
                    ↓
   输入完全相同   →  算法之间的差异 = 算法本身的差异
   多个包        →  看得出算法在不同数据上稳不稳（②可重复性）
   可反复回放    →  同一个包还能跑很多遍
```

实时跑的话，每次人走的路径/速度都不同，**没法横向比**。
