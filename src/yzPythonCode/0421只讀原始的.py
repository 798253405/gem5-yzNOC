import os
import numpy as np
import pandas as pd

# 基础路径和配置
#base_directory = "/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/yzRLPython/oneActionAfterBootOS/"
#starttick = 334516476158500
base_directory = "/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/yzRLPython/oneAction/"
starttick = 54427247860000



num_epochs = 500
readFOLDER_ACTION = 1.01#2025#0.82025

cycle_interval = 20000 * 500
analyzinglineNumbInOneFile = 20
totalLinesInOneFile = 20
num_nodes = 64
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
columns = ['timestamp', 'injreq', 'thres', 'injcount', 'Inj', 'Token', 'Quedelay', 'netdelay', 'Throughput']
df = pd.DataFrame(avg_all_epochs_data_nostring_noclevelavgdata, columns=columns)

print("\n每个 epoch 的平均值：")
print(df)

#%%
import matplotlib.pyplot as plt
import matplotlib
plt.plot(df.index, df['Throughput'], marker='.', linestyle='-', label='Average Throughput')
# 添加標籤和標題
plt.xlabel("  (Epoch)")
plt.ylabel("  (Average Throughput)")
plt.title("(Average Throughput vs (Epoch)")

# 添加網格線
plt.grid(True)

# 添加圖例（如果有多條線，比如繪製其他指標時）
plt.legend()

# 顯示圖表
plt.show()














