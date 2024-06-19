#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import ast
import numpy as np
# Function to read and process CSV data
def read_csv(file_path):
    df = pd.read_csv(file_path)
    data = []

    # Iterate through each cell and extract coordinates
    for row in df.itertuples(index=False):
        row_data = []
        for cell in row:
            # Convert string representation of list to actual list
            coordinates = ast.literal_eval(cell)
            row_data.append(coordinates)
        data.append(row_data)
    return data

# Function to prepare data for plotting
def prepare_data(data):
    x_coords = []
    y_coords = []

    for row in data:
        for coords in row:
            x_coords.append(coords[1])
            # y_coords.append(coords[0]- (0.762*3)/2)
            y_coords.append(coords[0]- (0.762 * 3)/2)
    return x_coords, y_coords

# Read and process data from both CSV files

# data1 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_-30cm_corn.csv')
data1 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_-20cm_corn.csv')
# data3 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_-10cm_corn.csv')
# data4 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_0cm_corn.csv')
# data5 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_10cm_corn.csv')
# data6 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_20cm_corn.csv')
# data7 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_30cm_corn.csv')

# data1 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/PP_deviate_0cm_corn.csv')
# data2 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/PP_deviate_10cm_corn.csv')
# data3 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/PP_deviate_20cm_corn.csv')
# data4 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_0cm_corn.csv')
# data5 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_10cm_corn.csv')
# data6 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_20cm_corn.csv')
# data7 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/Mpc_deviate_30cm_corn.csv')
# Prepare data for plotting
# data1 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/visual_servoing_0cm_corn.csv')
# data2 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/visual_servoing_-10cm_corn.csv')
# data3 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/visual_servoing_10cm_corn.csv')
# data4 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/visual_servoing_20cm_corn.csv')


data2 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/0.1_covariance_corn_old.csv')
data3 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/0.1_covariance_corn_new.csv')
data4 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/0.2_covariance_corn_new.csv')
data5 = read_csv('/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/0.2_covariance_corn_old.csv')
x1, y1 = prepare_data(data1)
x2, y2 = prepare_data(data2)
x3, y3 = prepare_data(data3)
x4, y4 = prepare_data(data4)
x5, y5 = prepare_data(data5)
# x6, y6 = prepare_data(data6)
# x7, y7 = prepare_data(data7)
# Plotting
plt.figure(figsize=(10, 6))

# plt.plot(x1, y1, c='blue', label='MPC deviate -30cm')
# plt.plot(x2, y2, c='red', label='MPC deviate -20cm')
# plt.plot(x3, y3, c='green', label='MPC deviate -10cm')
# plt.plot(x4, y4, c='purple', label='MPC deviate 0cm')
# plt.plot(x5, y5, c='black', label='MPC deviate 10cm')
# plt.plot(x6, y6, c='orange', label='MPC deviate 20cm')
# plt.plot(x7, y7, c='brown', label='MPC deviate 30cm')
# plt.plot(x1, y1, c='blue', label='PP deviate 0cm')
# plt.plot(x2, y2, c='red', label='PP deviate 10cm')
# plt.plot(x3, y3, c='green', label='PP deviate 20cm')
# plt.plot(x1, y1, c='blue', label='Visual servoing deviate 0cm')
# plt.plot(x2, y2, c='red', label='Visual servoing -10cm')
# plt.plot(x3, y3, c='green', label='Visual servoing 10cm')
# plt.plot(x4, y4, c='purple', label='Visual servoing deviate 20cm')
# plt.plot(x1, y1, c='blue', label='Visual servoing deviate 0cm')
plt.plot(x2, y2, c='red', label='Visual servoing -10cm')
# plt.plot(x3, y3, c='green', label='Visual servoing 10cm')
# plt.plot(x4, y4, c='purple', label='Visual servoing deviate 20cm')
plt.plot(x5, y5, c='black', label='MPC deviate 10cm')
# plt.plot(x6, y6, c='orange', label='MPC deviate 10cm')
# plt.plot(x7, y7, c='brown', label='MPC deviate 30cm')

plt.xlabel('Travel Distance')
plt.ylabel('Error')
plt.title('Plot of deviation from the center line')
plt.legend()
plt.grid(True)
plt.show()