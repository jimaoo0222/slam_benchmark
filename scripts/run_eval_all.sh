#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  采完数据后 · 一键评测
#
#  用法：
#     bash scripts/run_eval_all.sh <实验目录>
#
#  例：
#     bash scripts/run_eval_all.sh results/2026-10-10_场地A
#
#  它会自动做：
#     1. 检查真值文件 / 轨迹文件是否齐全
#     2. 对每条 traj_*.txt 跑评测（两组误差：不对齐 + 对齐）
#     3. 如果目录里有 scenes.csv，同时算「退化比」
#     4. 结果图和输出分别存到 eval_<名字>/ 目录
# ══════════════════════════════════════════════════════════════
set -uo pipefail

DIR="${1:-}"
if [[ -z "$DIR" ]]; then
  cat <<'EOF'
用法: bash scripts/run_eval_all.sh <实验目录>

例:   bash scripts/run_eval_all.sh results/2026-10-10_场地A

目录里应该有:
    gt.txt          真值（卷尺量 + 停车时刻）
    traj_*.txt      各算法输出的轨迹
    scenes.csv      （可选）场景分段，有它才能算退化比
EOF
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL="$HERE/benchmark_eval.py"
PY="${PY:-python3}"

cd "$DIR" || { echo "❌ 目录不存在: $DIR"; exit 1; }
echo "实验目录: $(pwd)"
echo

# ── 1. 真值文件 ────────────────────────────────────────────
if [[ ! -f gt.txt ]]; then
  echo "❌ 缺少 gt.txt"
  echo "   从模板复制一份:  cp ../_TEMPLATE/gt.txt ."
  exit 1
fi
echo "✅ gt.txt"

# 检查时间戳是否还没填（全为 0 说明只用 --find-stops 之前的模板）
if awk '!/^#/ && NF>=8 && $1=="0.000" {c++} END{exit !(c>0)}' gt.txt; then
  echo "⚠️  gt.txt 里还有 0.000 的时间戳 —— 真值点会被跳过"
  echo "   先跑:  $PY $EVAL --find-stops traj_lidar.txt"
  echo "   把生成的时间戳填进 gt.txt，再运行本脚本"
  echo
fi

# ── 2. 找轨迹 ──────────────────────────────────────────────
shopt -s nullglob
TRAJS=(traj_*.txt)
shopt -u nullglob

if [[ ${#TRAJS[@]} -eq 0 ]]; then
  echo "❌ 没找到任何 traj_*.txt"
  echo "   把各算法输出的轨迹放进来，文件名以 traj_ 开头"
  echo "   例: traj_fastlio2.txt / traj_pointlio.txt / traj_glim.txt"
  exit 1
fi
echo "✅ 找到 ${#TRAJS[@]} 条轨迹: ${TRAJS[*]}"
echo

# ── 3. 场景分段（可选）──────────────────────────────────────
SEG=()
if [[ -f scenes.csv ]]; then
  SEG=(--segments scenes.csv)
  echo "✅ scenes.csv —— 会同时算「退化比」"
else
  echo "⚠️  没有 scenes.csv —— 只出总体误差，算不出退化比"
  echo "   想算退化比就从模板复制:  cp ../_TEMPLATE/scenes.csv ."
fi
echo

# ── 4. 逐条跑 ──────────────────────────────────────────────
for t in "${TRAJS[@]}"; do
  name="${t#traj_}"
  name="${name%.txt}"
  echo "════════════════════════════════════════════════════════"
  echo "  评测算法的轨迹:  $name"
  echo "════════════════════════════════════════════════════════"
  "$PY" "$EVAL" --gt gt.txt --est "$t" --outdir "eval_$name" "${SEG[@]}"
  echo
done

echo "════════════════════════════════════════════════════════"
echo "✅ 全部完成"
echo
echo "   结果图:  eval_<名字>/benchmark_report.png"
echo "   填表用:  上面每个算法的「不对齐 / 对齐」中位数 → 对比表.md"
echo "════════════════════════════════════════════════════════"
