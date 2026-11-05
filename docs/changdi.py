import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import platform

# --- Font Settings (Pure English / Pinyin) ---
system = platform.system()
if system == 'Windows':
    plt.rcParams['font.sans-serif'] = ['SimHei']
elif system == 'Darwin':
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS']
else: # Linux
    linux_fonts = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Source Han Sans CN', 
                   'WenQuanYi Zen Hei', 'SimHei', 'Microsoft YaHei']
    available_fonts = set([f.name for f in fm.fontManager.ttflist])
    
    found_font = None
    for font in linux_fonts:
        if font in available_fonts:
            found_font = font
            break
            
    if found_font:
        plt.rcParams['font.sans-serif'] = [found_font]
    else:
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans']

plt.rcParams['axes.unicode_minus'] = False

# --- Coordinates ---
x0, y0 = 0, 0
x1, y1 = 0, 180      
x2, y2 = -180, 0
x3, y3 = -(180 + 225), 0
x4, y4 = -(180 + 225 + 225), 0

# --- Create Canvas ---
fig, ax = plt.subplots(figsize=(10, 5))

# --- Draw Lines ---
ax.plot([x4, x0], [y0, y0], color='black', linestyle='--', linewidth=2, zorder=1)
ax.plot([x0, x1], [y0, y1], color='gray', linestyle='--', linewidth=2, zorder=1)
ax.text(10, 90, '180cm', fontsize=12, verticalalignment='center', color='gray')

# --- Draw Nodes and Labels ---
# Node 4
ax.plot(x4, y4, marker='|', markersize=15, color='black', markeredgewidth=2)
ax.text(x4 - 10, y4 - 20, '4', fontsize=14, horizontalalignment='center')

# Node 3
ax.plot(x3, y3, marker='+', markersize=15, color='black', markeredgewidth=2)
ax.text(x3, y3 - 25, '3', fontsize=14, horizontalalignment='center')

# Node 2
ax.plot(x2, y2, marker='+', markersize=15, color='black', markeredgewidth=2)
ax.text(x2, y2 - 25, '2', fontsize=14, horizontalalignment='center')

# Node 0
ax.plot(x0, y0, marker='o', markersize=8, color='black')
ax.text(x0 + 15, y0 - 10, '0', fontsize=14)

# Node 1
ax.plot(x1, y1, marker='o', markersize=8, color='black', zorder=3)
ax.text(x1 + 15, y1, '1', fontsize=14, verticalalignment='center') 

# Arrows at Node 1
ax.annotate('', xy=(x1, y1 + 30), xytext=(x1, y1 + 10), 
            arrowprops=dict(arrowstyle='->', color='black'))
ax.annotate('', xy=(x1, y1 - 30), xytext=(x1, y1 - 10), 
            arrowprops=dict(arrowstyle='->', color='black'))

# --- Distance Dimensions ---
ax.text((x4 + x3)/2, 15, '225cm', fontsize=12, horizontalalignment='center')
ax.text((x3 + x2)/2, 15, '225cm', fontsize=12, horizontalalignment='center')
ax.text((x2 + x0)/2, 15, '180cm', fontsize=12, horizontalalignment='center')

# --- Formatting ---
# Changed title to Pinyin
ax.set_title('Changdi Fenbu Shiyitu', fontsize=16, pad=20)
ax.set_xlim(x4 - 50, x0 + 100) 
ax.set_ylim(-50, 280)          
ax.axis('off')                

total_length = abs(x4)
# Changed text to English
ax.text((x4 + x0)/2, -40, f'Total Horizontal Span: {total_length}cm', 
        fontsize=12, horizontalalignment='center', color='blue', fontweight='bold')

# --- Show and Save ---
plt.tight_layout()
plt.savefig('site_layout.png', dpi=300, bbox_inches='tight')
plt.show()