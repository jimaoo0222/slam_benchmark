import matplotlib.pyplot as plt

# 1. 定义坐标 (X, Y)，单位：cm
points = {
    '0': (0, 0),
    '1': (0, 315),
    '2': (0, 913.09),
    '3': (0, 1138.09),
    '4': (270, 1138.09),
    '5': (270, 0)
}

x_coords = [points[str(i)][0] for i in range(6)] + [points['0'][0]]
y_coords = [points[str(i)][1] for i in range(6)] + [points['0'][1]]

# 2. 创建画布
fig, ax = plt.subplots(figsize=(8, 10))

# 3. 画线和点
ax.plot(x_coords, y_coords, color='black', linewidth=2, zorder=1)
ax.scatter(x_coords[:-1], y_coords[:-1], color='red', facecolors='white', s=150, linewidth=2, zorder=2)

# 4. 添加点标号 (0 到 5)
for i in range(6):
    offset_x = -25 if i in [0, 1, 2, 3] else 15
    offset_y = -35 if i in [0, 5] else 15
    ax.text(points[str(i)][0] + offset_x, points[str(i)][1] + offset_y, 
            str(i), fontsize=16, fontweight='bold')

# 5. 添加横向的文字标注
# 顶部
ax.text(135, 1150, '270cm (6x45cm)', ha='center', va='bottom', fontsize=12)
# 左侧 (点0-1)
ax.text(-20, 315/2, '7x45cm\n315cm', ha='right', va='center', fontsize=12)
# 左侧 (点1-2)
ax.text(-20, (315 + 913.09)/2, '13.09+13x45cm\n598.09cm', ha='right', va='center', fontsize=12)
# 左侧 (点2-3)
ax.text(-20, (913.09 + 1138.09)/2, '5x45cm\n225cm', ha='right', va='center', fontsize=12)
# 右侧 (点4-5)
ax.text(290, 1138.09/2, '13.09+25x45cm\n1138.09cm', ha='left', va='center', fontsize=12)
# 底部 (改为 270cm)
ax.text(135, -60, '270cm (6x45cm)', ha='center', va='top', fontsize=12)

# 6. 美化与隐藏坐标轴
ax.set_aspect('equal') # 保持 1:1 比例
ax.axis('off')         # 隐藏 X 轴、Y 轴、刻度和边框

# 设置显示范围，留出文字空间，防止底部文字被裁切
ax.set_xlim(-250, 550)
ax.set_ylim(-150, 1250)

# 7. 保存并显示
filename = 'clean_path_plot.png'
plt.savefig(filename, dpi=300, bbox_inches='tight', transparent=False)
print(f"Image successfully saved as: {filename}")
plt.show()