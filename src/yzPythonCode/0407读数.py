import os
import numpy as np
import pandas as pd

# 基础路径和配置
base_directory = "/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/yzRLPython/oneAction/"
num_epochs = 10

readFOLDER_ACTION = 1.01#2025#0.82025
starttick = 334516476158500
cycle_interval = 20000 * 500
analyzinglineNumbInOneFile = 1
totalLinesInOneFile = 20
num_nodes = 128
directory = os.path.join(base_directory, f"inj_{readFOLDER_ACTION}")

# 存放所有 epoch 的原始字符串数据
all_epochs_data = []

for epoch_index in range(num_epochs):
    #print(f"  Processing epoch {epoch_index + 1}/{num_epochs}")
    epoch_data = []
    current_cycle = epoch_index * cycle_interval

    for node_index in range(num_nodes):
        filename = f"{current_cycle + starttick}_node_{node_index}.txt"
        filepath = os.path.join(directory, filename)

        # 改为只保留 5 行
        node_cycles = [""] * analyzinglineNumbInOneFile

        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                if ((totalLinesInOneFile- analyzinglineNumbInOneFile)==0 ):
                    last_lines = lines[-totalLinesInOneFile: ]
                else    :
                    last_lines = lines[-totalLinesInOneFile:-(totalLinesInOneFile- analyzinglineNumbInOneFile)]
                start_index = analyzinglineNumbInOneFile - len(last_lines)
                for i in range(len(last_lines)):
                    node_cycles[start_index + i] = last_lines[i].strip()
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"    Error reading file: {filepath} - {e}")

        epoch_data.append(node_cycles)

    if any(any(cycle != "" for cycle in node) for node in epoch_data):
        all_epochs_data.append(epoch_data)

print("Finished read_node_data")

# 提取数字函数
def extract_numbers_from_line(line_string: str) -> list:
    if not line_string or not line_string.strip():
        return []
    extracted_numbers = []
    parts = line_string.split(',')

    try:
        timestamp_str = parts[0].strip()
        extracted_numbers.append(float(timestamp_str))
    except (ValueError, IndexError):
        extracted_numbers.append(None)

    for i in range(1, 9):
        value = None
        if i < len(parts):
            part = parts[i].strip()
            if part:
                try:
                    split_part = part.split()
                    if split_part:
                        value_str = split_part[-1]
                        value = float(value_str)
                except (ValueError, IndexError):
                    pass
        extracted_numbers.append(value)

    return extracted_numbers[:9]

# 将字符串转为数字列表
all_epochs_data_nostring = []
for onefiledata in all_epochs_data:
    onefiledata_nostring = []
    for node_index in range(num_nodes):
        node_data = onefiledata[node_index]
        nodedata_nostring = []
        for onelinedata in node_data:
            onelinedata = extract_numbers_from_line(onelinedata)
            nodedata_nostring.append(onelinedata)
        onefiledata_nostring.append(nodedata_nostring)
    all_epochs_data_nostring.append(onefiledata_nostring)

# 转为 numpy 数组并打印维度
data_array = np.array(all_epochs_data_nostring, dtype=np.float32)
print("data_array.shape:", data_array.shape)  # 应为 (epoch, 128, 5, 9)

# 对128个节点取平均 → (epoch, 5, 9)
avg_all_epochs_data_nostring = np.nanmean(data_array, axis=1)

# 对5行时间点取平均 → (epoch, 9)
avg_all_epochs_data_nostring_noclevelavgdata = np.nanmean(avg_all_epochs_data_nostring, axis=1)

# 转为 pandas DataFrame
columns = ['timestamp', 'v1', 'v2', 'v3', 'v4', 'v5', 'v6', 'v7', 'v8']
df = pd.DataFrame(avg_all_epochs_data_nostring_noclevelavgdata, columns=columns)

print("\n每个 epoch 的平均值：")
print(df)





























readFOLDER_ACTION = 0.5
totalLinesInOneFile = 20
directory = os.path.join(base_directory, f"inj_{readFOLDER_ACTION}")
# 存放所有 epoch 的原始字符串数据
all_epochs_data_AD = []

for epoch_index in range(num_epochs):
    #print(f"  Processing epoch {epoch_index + 1}/{num_epochs}")
    epoch_data = []
    current_cycle = (epoch_index) * cycle_interval

    for node_index in range(num_nodes):
        filename = f"{current_cycle + starttick}_node_{node_index}.txt"
        filepath = os.path.join(directory, filename)

        # 改为只保留 5 行
        node_cycles = [""] * analyzinglineNumbInOneFile

        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                if ((totalLinesInOneFile - analyzinglineNumbInOneFile) == 0):
                    last_lines = lines[-totalLinesInOneFile:]
                else:
                    last_lines = lines[-totalLinesInOneFile:-(totalLinesInOneFile - analyzinglineNumbInOneFile)]
                start_index = analyzinglineNumbInOneFile - len(last_lines)
                for i in range(len(last_lines)):
                    node_cycles[start_index + i] = last_lines[i].strip()
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"    Error reading file: {filepath} - {e}")

        epoch_data.append(node_cycles)

    if any(any(cycle != "" for cycle in node) for node in epoch_data):
        all_epochs_data_AD.append(epoch_data)

print("Finished read_node_data")

# 提取数字函数
def extract_numbers_from_line(line_string: str) -> list:
    if not line_string or not line_string.strip():
        return []
    extracted_numbers = []
    parts = line_string.split(',')

    try:
        timestamp_str = parts[0].strip()
        extracted_numbers.append(float(timestamp_str))
    except (ValueError, IndexError):
        extracted_numbers.append(None)

    for i in range(1, 9):
        value = None
        if i < len(parts):
            part = parts[i].strip()
            if part:
                try:
                    split_part = part.split()
                    if split_part:
                        value_str = split_part[-1]
                        value = float(value_str)
                except (ValueError, IndexError):
                    pass
        extracted_numbers.append(value)

    return extracted_numbers[:9]

# 将字符串转为数字列表
all_epochs_data_nostring_AD = []
for onefiledata in all_epochs_data_AD:
    onefiledata_nostring = []
    for node_index in range(num_nodes):
        node_data = onefiledata[node_index]
        nodedata_nostring = []
        for onelinedata in node_data:
            onelinedata = extract_numbers_from_line(onelinedata)
            nodedata_nostring.append(onelinedata)
        onefiledata_nostring.append(nodedata_nostring)
    all_epochs_data_nostring_AD.append(onefiledata_nostring)

# 转为 numpy 数组并打印维度
data_array_AD = np.array(all_epochs_data_nostring_AD, dtype=np.float32)
print("data_array.shape:", data_array_AD.shape)  # 应为 (epoch, 128, 5, 9)

# 对128个节点取平均 → (epoch, 5, 9)
avg_all_epochs_data_nostring_AD = np.nanmean(data_array_AD, axis=1)

# 对5行时间点取平均 → (epoch, 9)
avg_all_epochs_data_nostring_noclevelavgdata_AD= np.nanmean(avg_all_epochs_data_nostring_AD, axis=1)

# 转为 pandas DataFrame
columns = ['timestamp', 'v1', 'v2', 'yzPeriodActualInjPacketCount', 'v4', 'v5', 'v6', 'v7', 'yzPacketPeriodCountreceived']
df = pd.DataFrame(avg_all_epochs_data_nostring_noclevelavgdata_AD, columns=columns)

print("\n每个 epoch 的平均值：")
print(df)

#%%
# 如需保存 CSV：
# df.to_csv("epoch_avg_summary.csv", index=False)
import  matplotlib.pyplot as plt
plt.plot(avg_all_epochs_data_nostring_noclevelavgdata[:, 8], label='avg_all_epochs_data_nostring_noclevelavgdata[:, 0]')
plt.plot(avg_all_epochs_data_nostring_noclevelavgdata_AD[:, 8], label='ad 0.8')
plt.legend()
plt.show()
#%%
import matplotlib.ticker as mticker
data_a = avg_all_epochs_data_nostring_noclevelavgdata[:, 8]
data_b = avg_all_epochs_data_nostring_noclevelavgdata_AD[:, 8]
# 1. 创建 X 轴数据 (使用索引)
x = np.arange(len(data_a))
# 2. 计算百分比差异
epsilon = 1e-9 # 防止除以零
percentage_diff = np.where(
    np.abs(data_a) > epsilon,
    ((data_b - data_a) / data_a) * 100,
    0
)
# 3. 创建图形和子图 (2行1列，共享X轴)
fig, axes = plt.subplots(
    2, 1,
    sharex=True,
    figsize=(10, 7), # 您可以调整图形大小
    gridspec_kw={'height_ratios': [1, 2]} # 上1下2的高度比例
)

# 移除子图间的垂直间距
fig.subplots_adjust(hspace=0)

# --- 绘制最上面的百分比差异图 (axes[0]) ---
ax0 = axes[0]
ax0.plot(x, percentage_diff, label='(Series A - Series B) / B %', color='red') # 您可以修改 label
ax0.axhline(0, color='grey', linestyle='--', linewidth=0.8)
ax0.set_ylabel('Difference (%)')
ax0.set_title('Top: Percentage Difference / Bottom: Series Values')
ax0.legend(loc='upper left')
ax0.grid(True, linestyle='--', alpha=0.6)
ax0.yaxis.set_major_formatter(mticker.PercentFormatter())

# --- 绘制下方的两个系列合并图 (axes[1]) ---
ax1 = axes[1]
# 使用您代码中的数据和标签 (注意第一个label可能需要修正)
ax1.plot(x, data_a, label='default Series (Col 8)', color='blue') # 注意: 您的原始label是'avg_all_epochs_data_nostring_noclevelavgdata[:, 0]'，但数据是[:, 8]
ax1.plot(x, data_b, label='our method ad 0.8', color='green') # 使用您代码中的label
ax1.set_ylabel('Value')
ax1.set_xlabel('Epoch / Index') # 根据您的X轴含义修改
ax1.legend(loc='upper left')
ax1.grid(True, linestyle='--', alpha=0.6)

# 调整整体布局
plt.tight_layout(h_pad=0)

# 显示图形
plt.show()



timeAdWorkCount = 0
for i in range(avg_all_epochs_data_nostring_noclevelavgdata.shape[0]):
    if avg_all_epochs_data_nostring_noclevelavgdata[i, 8] < avg_all_epochs_data_nostring_noclevelavgdata_AD[i, 8]:
        timeAdWorkCount += 1
        print("fileid=",i," ", avg_all_epochs_data_nostring_noclevelavgdata[i, 8]," ", avg_all_epochs_data_nostring_noclevelavgdata_AD[i, 8])
#%% plt单独的fileepochs
# extracted_values = all_epochs_data_nostring[cycle_index, :, line_index, value_index] #7 22 35 36 48
#for i in range(num_epochs):
for i in range(21,23):
    extracted_values = np.array(all_epochs_data_nostring)[i, :, 0,3]
    #extracted_values_nextepochs = np.array(all_epochs_data_nostring)[i, :, 1, 3]
    extracted_values_AD = np.array(all_epochs_data_nostring_AD)[i, :, 0, 3]
   # extracted_values_nextepochs_AD = np.array(all_epochs_data_nostring_AD)[i, :, 1, 3]
    plt.plot(extracted_values, label='no AD, default')
    #plt.plot(extracted_values_nextepochs, label='all_epochs_data_nostring[22, :, 1, 8]')
    plt.plot(extracted_values_AD, label=' AD[22, :, 0, 8]')
    #plt.plot(extracted_values_nextepochs_AD, label=' AD[22, :, 1, 8]')

    plt.plot(np.array(all_epochs_data_nostring_AD)[i, :, 0, 3],label=' ad+ actualInjPacketCount')


    plt.legend()
    plt.show()