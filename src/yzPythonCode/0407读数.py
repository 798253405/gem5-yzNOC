import os
import numpy as np
import pandas as pd

# 基础路径和配置
base_directory = "/home/yz/myprojects/2024GEM5/parsec-tests/yzmodifiedgem5/yzRLPython/oneAction/"
num_epochs = 31

readFOLDER_ACTION = 1.01#2025#0.82025
starttick = 334516476158500
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
columns = ['timestamp', 'v1', 'v2', 'v3', 'v4', 'v5', 'v6', 'v7', 'v8']
df = pd.DataFrame(avg_all_epochs_data_nostring_noclevelavgdata, columns=columns)

print("\n每个 epoch 的平均值：")
print(df)





























readFOLDER_ACTION = 1.2
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
#只是爲了看圖方便！

# for i in range(len(percentage_diff)):
#   if percentage_diff[i] < 0:
#     percentage_diff[i] = 0
count_positive = sum(1 for num in percentage_diff if num > 0)
count_significant_positiveRate = sum(1 for num in percentage_diff if num > 2) / len(percentage_diff)
positive_numbers = [num for num in percentage_diff  if num > 0]
if (len(positive_numbers) > 0):
    average_of_positives = sum(positive_numbers) / len(positive_numbers)
    print("count_positive / len(percentage_diff)= ",count_positive / len(percentage_diff), "average_of_positives= ",average_of_positives,"count_significant_positiveRate= ",count_significant_positiveRate)
else:
    print("len(positive_numbers) = 0")
##只是爲了看圖方便！結束
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

    # ===========================================================
    # 新增：讀取和處理 inj_0.5 的數據
    # ===========================================================
    print("\n" + "=" * 30 + "\nStarting processing for inj_0.5\n" + "=" * 30)  # 添加分隔符和提示信息





















#%%

############%
readFOLDER_ACTION_05 = 0.5  # 新增：指定 0.5 的文件夾標識
directory_05 = os.path.join(base_directory, f"inj_{readFOLDER_ACTION_05}")  # 新增：0.5 的目錄路徑
# 存放所有 epoch 的原始字符串数据 (for 0.5)
all_epochs_data_05 = []  # 新增：使用新列表名

for epoch_index in range(num_epochs):
    # print(f"  Processing epoch {epoch_index + 1}/{num_epochs} for 0.5") # 可選的打印信息
    epoch_data = []
    # 注意：這裡 current_cycle 的計算方式需要與你的實驗設計一致
    # 如果 0.5 和 0.95 的 epoch 起始時間點計算方式相同，則保持 current_cycle 的計算方式
    current_cycle = (epoch_index) * cycle_interval  # 假設與 0.95 的計算方式相同

    for node_index in range(num_nodes):
        filename = f"{current_cycle + starttick}_node_{node_index}.txt"
        filepath = os.path.join(directory_05, filename)  # 使用 directory_05

        node_cycles = [""] * analyzinglineNumbInOneFile

        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                # 保持與之前相同的行讀取邏輯
                if ((totalLinesInOneFile - analyzinglineNumbInOneFile) == 0):
                    last_lines = lines[-totalLinesInOneFile:]
                else:
                    last_lines = lines[-totalLinesInOneFile:-(totalLinesInOneFile - analyzinglineNumbInOneFile)]
                start_index = analyzinglineNumbInOneFile - len(last_lines)
                for i in range(len(last_lines)):
                    node_cycles[start_index + i] = last_lines[i].strip()
        except FileNotFoundError:
            # print(f"    File not found (0.5): {filepath}") # 可選打印
            pass
        except Exception as e:
            print(f"    Error reading file (0.5): {filepath} - {e}")

        epoch_data.append(node_cycles)

    # 檢查是否有有效數據才添加
    if any(any(cycle != "" for cycle in node) for node in epoch_data):
        all_epochs_data_05.append(epoch_data)  # 添加到 _05 列表

print("Finished read_node_data for data-C {readFOLDER_ACTION_05}")

# --- 將字符串轉為數字列表 (for 0.5) ---
# 注意：這裡使用了之前定義的 extract_numbers_from_line 函數
all_epochs_data_nostring_05 = []  # 新增變量名
for onefiledata in all_epochs_data_05:  # 使用 _05 數據
    onefiledata_nostring = []
    for node_index in range(num_nodes):
        node_data = onefiledata[node_index]
        nodedata_nostring = []
        for onelinedata in node_data:
            # 確保 extract_numbers_from_line 函數在你添加此代碼塊時仍然可用
            onelinedata = extract_numbers_from_line(onelinedata)
            nodedata_nostring.append(onelinedata)
        onefiledata_nostring.append(nodedata_nostring)
    all_epochs_data_nostring_05.append(onefiledata_nostring)  # 添加到 _05 列表

# --- 轉為 numpy 数组并打印维度 (for DATA-C) ---
data_array_05 = np.array(all_epochs_data_nostring_05, dtype=np.float32)  # 新增變量名
print("data_array_05.shape:", data_array_05.shape)  # 打印新數組的維度

# --- 对节点取平均 (for 0.5) ---
avg_all_epochs_data_nostring_05 = np.nanmean(data_array_05, axis=1)  # 新增變量名

# --- 对时间点取平均 (for 0.5) ---
avg_all_epochs_data_nostring_noclevelavgdata_05 = np.nanmean(avg_all_epochs_data_nostring_05, axis=1)  # 新增變量名

# --- 转为 pandas DataFrame (for 0.5) ---
# 注意：這裡的 columns 變量需要與你的 0.5 數據列對應，如果列名不同需要調整
# 假設列名與之前的相同
columns_05 = ['timestamp', 'v1', 'v2', 'yzPeriodActualInjPacketCount', 'v4', 'v5', 'v6', 'v7',
              'yzPacketPeriodCountreceived']  # 可以根據需要修改
df_05 = pd.DataFrame(avg_all_epochs_data_nostring_noclevelavgdata_05, columns=columns_05)  # 新增 DataFrame

print("\n每个 epoch 的平均值 (for dataC {readFOLDER_ACTION_05}):")
print(df_05)
print("\n" + "=" * 30 + "\nFinished processing for inj_0.5\n" + "=" * 30)
# ===========================================================
# 新增：计算 0.5 vs 1.01 的百分比差异和统计
# ===========================================================
print("\n" + "="*30 + "\nStatistics for dataC vs 1.01\n" + "="*30)

# 提取 1.01 (baseline) 和 0.5 的第 8 列数据
data_a = avg_all_epochs_data_nostring_noclevelavgdata[:, 8] # Baseline (1.01) - 保持不變
data_c = avg_all_epochs_data_nostring_noclevelavgdata_05[:, 8] # 新数据 (0.5)

# 确保数据长度一致 (如果 epoch 数量不同，可能需要处理)
min_len_05 = min(len(data_a), len(data_c))
data_a_trimmed_05 = data_a[:min_len_05]
data_c_trimmed = data_c[:min_len_05]
print(f"Warning: Trimming data for data-C comparison to {min_len_05} epochs if lengths differ.")

# 计算百分比差异 (0.5 vs 1.01)
epsilon = 1e-9 # 防止除以零
percentage_diff_05 = np.where(
    np.abs(data_a_trimmed_05) > epsilon,
    ((data_c_trimmed - data_a_trimmed_05) / data_a_trimmed_05) * 100,
    0
)

# 计算统计数据 (0.5 vs 1.01)
if len(percentage_diff_05) > 0:
    count_positive_05 = sum(1 for num in percentage_diff_05 if num > 0)
    proportion_positive_05 = count_positive_05 / len(percentage_diff_05)

    # 计算提升 > 2% 的占比
    count_significant_positive_05 = sum(1 for num in percentage_diff_05 if num > 2)
    count_significant_positiveRate_05 = count_significant_positive_05 / len(percentage_diff_05)

    # 计算正提升部分的平均值
    positive_numbers_05 = [num for num in percentage_diff_05 if num > 0]
    if len(positive_numbers_05) > 0:
        average_of_positives_05 = sum(positive_numbers_05) / len(positive_numbers_05)
    else:
        average_of_positives_05 = 0 # 或者 np.nan

    print(f"Proportion of epochs where data-C > 1.01: {proportion_positive_05:.4f} ({proportion_positive_05*100:.2f}%)")
    print(f"Average percentage increase (when dataC  > 1.01): {average_of_positives_05:.4f}%")
    print(f"Proportion of epochs where DATA-C has >2% increase vs 1.01: {count_significant_positiveRate_05:.4f} ({count_significant_positiveRate_05*100:.2f}%)")
else:
    print("Could not calculate statistics for DATA-C vs 1.01 (possibly empty data).")

print("\n" + "="*30 + "\nEnd Statistics for DATA-C vs 1.01\n" + "="*30)
# 在你的繪圖代碼塊中:
# 找到 fig, axes = plt.subplots(...) 這一行

# --- 准备绘图数据 ---
data_a = avg_all_epochs_data_nostring_noclevelavgdata[:, 8] # Baseline 1.01
data_b = avg_all_epochs_data_nostring_noclevelavgdata_AD[:, 8] # Comparison 0.95
data_c = avg_all_epochs_data_nostring_noclevelavgdata_05[:, 8] # 新增：Comparison 0.5

# 处理可能的数据长度不一致问题
min_len = min(len(data_a), len(data_b), len(data_c))
x = np.arange(min_len) # X 轴使用最小长度
data_a = data_a[:min_len]
data_b = data_b[:min_len]
data_c = data_c[:min_len]
print(f"Plotting data trimmed to {min_len} epochs.")

# 重新计算百分比差异 (基于裁剪后的数据)
epsilon = 1e-9
percentage_diff_095 = np.where( # 重命名，更清晰
    np.abs(data_a) > epsilon,
    ((data_b - data_a) / data_a) * 100,
    0
)
percentage_diff_05 = np.where( # 重命名，更清晰
    np.abs(data_a) > epsilon,
    ((data_c - data_a) / data_a) * 100,
    0
)


# --- 创建图形和子图 (保持不变) ---
fig, axes = plt.subplots(
    2, 1,
    sharex=True,
    figsize=(12, 8), # 可以调整图形大小
    gridspec_kw={'height_ratios': [1, 2]} # 上1下2的高度比例
)
fig.subplots_adjust(hspace=0) # 移除子图间的垂直间距

# --- 绘制最上面的百分比差异图 (axes[0]) ---
ax0 = axes[0]
# 绘制 0.95 vs 1.01 的差异
ax0.plot(x, percentage_diff_095, label='Diff% (0.95 vs 1.01)', color='red', linestyle='-') # 修改 label
# 新增：绘制 0.5 vs 1.01 的差异
ax0.plot(x, percentage_diff_05, label='Diff% (0.5 vs 1.01)', color='orange', linestyle=':') # 新增线条，使用不同颜色和线型
ax0.axhline(0, color='grey', linestyle='--', linewidth=0.8)
ax0.set_ylabel('Difference (%)')
ax0.set_title('Top: Percentage Difference vs Baseline (1.01) / Bottom: Series Values')
ax0.legend(loc='upper left') # 图例会自动更新
ax0.grid(True, linestyle='--', alpha=0.6)
ax0.yaxis.set_major_formatter(mticker.PercentFormatter())

# --- 绘制下方的三个系列合并图 (axes[1]) ---
ax1 = axes[1]
ax1.plot(x, data_a, label='Baseline (1.01)', color='blue') # 修改 label
ax1.plot(x, data_b, label='Method 0.95', color='green') # 修改 label
# 新增：绘制 0.5 的数据
ax1.plot(x, data_c, label='Method 0.5', color='purple') # 新增线条，使用不同颜色
ax1.set_ylabel('Value (Column 8: yzPacketPeriodCountreceived)') # 明确 Y 轴含义
ax1.set_xlabel('Epoch / Index')
ax1.legend(loc='upper left') # 图例会自动更新
ax1.grid(True, linestyle='--', alpha=0.6)

# --- 调整布局和显示 (保持不变) ---
plt.tight_layout(h_pad=0)
plt.show()
# 在你的详细 Epoch 绘图循环中:
# for i in range(21, 23): # 或者你的循环范围

# ... 获取 extracted_values 和 extracted_values_AD 的代码保持不变 ...
# 新增：获取 0.5 的对应数据
# 注意：这里的索引 [i, :, 0, 3] 需要根据你想比较的具体行和列进行调整
try:
    # 使用原始未平均的数据进行比较（如果需要）
    # 确保 data_array_05 在此作用域内可用
    extracted_values_05 = data_array_05[i, :, 0, 3] # 假设比较第 0 行，第 3 列
    # 如果你想比较的是平均后的数据，可以使用：
    # extracted_values_05 = avg_all_epochs_data_nostring_05[i, 0, 3] # 假设比较第 0 行，第 3 列

    # 检查数据长度是否一致，如果不一致可能需要处理或绘图会出错
    # ... (如果需要，添加长度检查和处理) ...

    # 新增：在 plt.plot 中添加 0.5 的数据
    plt.plot(extracted_values_05, label=f'Method 0.5 Epoch {i} Col 3', linestyle='-.', alpha=0.8) # 添加 label 和不同线型

except IndexError:
    print(f"Warning: Could not plot detailed data for 0.5 in epoch {i}, possibly missing data.")
except NameError:
    print(f"Warning: data_array_05 not defined when plotting detailed epoch {i} data.")


# 在 plt.legend() 和 plt.show() 之前添加上面的 plt.plot(...)

# ... 你现有的 plt.legend() 和 plt.show() ...
# ===========================================================
# 新增：比较 0.5 vs 0.95 的数据点优劣次数
# ===========================================================
print("\n" + "="*30 + "\nComparison Count for 0.5 vs 0.95\n" + "="*30)

# 确认 data_b (代表 0.95) 和 data_c (代表 0.5) 已经基于 min_len 裁剪过
# 它们都是处理后的第 8 列数据

count_05_better_than_095 = 0 # 0.5 优于 0.95 的计数器
count_095_better_than_05 = 0 # 0.95 优于 0.5 的计数器

# 使用之前裁剪过的、长度一致的数据进行比较
if 'data_b' in locals() and 'data_c' in locals() and 'min_len' in locals():
    if len(data_b) >= min_len and len(data_c) >= min_len:
        for i in range(min_len): # 使用 min_len 确保索引有效
            # 检查并处理 NaN 值，如果 NaN 算作“不好”，则可以这样处理：
            val_b = data_b[i] if not np.isnan(data_b[i]) else -np.inf
            val_c = data_c[i] if not np.isnan(data_c[i]) else -np.inf

            if val_c > val_b: # 比较 Method 0.5 和 Method 0.95
                count_05_better_than_095 += 1
                # 可选：打印具体的优越点
                # print(f"Epoch {i}: Method(0.95)={data_b[i]:.4f}, Method(0.5)={data_c[i]:.4f} -> 0.5 is better")
            elif val_b > val_c: # 比较 Method 0.95 和 Method 0.5
                count_095_better_than_05 += 1
                # 可选：打印具体的优越点
                # print(f"Epoch {i}: Method(0.95)={data_b[i]:.4f}, Method(0.5)={data_c[i]:.4f} -> 0.95 is better")
            # else: # val_b == val_c (包括两个都是 NaN 或相等的情况)
            #     pass # 可以选择忽略或单独计数

        print(f"Number of epochs where Method DATA-C > Method DATA-B (Column 8): {count_05_better_than_095} out of {min_len} epochs")
        print(f"Number of epochs where Method DATA-B > Method DATA-C (Column 8): {count_095_better_than_05} out of {min_len} epochs")

        # (可选) 打印相等或均为无效值的次数
        count_equal_or_nan = min_len - count_05_better_than_095 - count_095_better_than_05
        print(f"Number of epochs where Method DATA-C == Method DATA-B or NaN involved (Column 8): {count_equal_or_nan} out of {min_len} epochs")

    else:
        print("Error: Data arrays (data_b, data_c) are shorter than min_len.")
else:
    print("Error: Required variables (data_b, data_c, min_len) not defined before comparison.")


print("\n" + "="*30 + "\nEnd Comparison Count for DATC vs DATAB\n" + "="*30)