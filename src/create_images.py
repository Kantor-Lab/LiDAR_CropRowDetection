#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3

import numpy as np
import matplotlib.pyplot as plt

# Function to simulate lidar point clouds
def generate_lidar_data(num_points=500):
    x = np.random.uniform(-10, 10, num_points)
    y = np.random.uniform(-10, 10, num_points)
    z = 0.1 * x + 0.2 * y + np.random.normal(0, 1, num_points)
    return x, y, z

# Generate lidar data
x, y, z = generate_lidar_data()

# Plot original lidar points
plt.scatter(x, y, c='blue', label='Original Lidar Points')

# Plot ground plane
ground_plane = np.linspace(-10, 10, 100)
plt.plot(ground_plane, 0.1 * ground_plane, 'r--')

# Filter points below ground plane
filtered_points = z > 0.1 * x + 0.2 * y
plt.scatter(x[filtered_points], y[filtered_points], c='red')

plt.title('Ground Plane Elevation')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()

# Show the plot
plt.show()


