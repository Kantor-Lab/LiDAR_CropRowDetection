import numpy as np
import open3d as o3d

# Your new data
data = """38.952,94.684,0,0,0,0
38.925,97.998,0,0,0,0
38.999,109.563,0,0,0,0
38.69,120.216,0,0,0,0
38.476,123.128,0,0,0,0
38.046,126.831,0,0,0,0
37.671,128.816,0,0,0,0
36.436,130.265,0,0,0,0
35.631,130.748,0,0,0,0
33.163,131.875,0,0,0,0
31.445,133.056,0,0,0,0
29.46,135.042,0,0,0,0
28.44,138.262,0,0,0,0
27.903,141.375,0,0,0,0
27.742,146.366,0,0,0,0
27.796,155.113,0,0,0,0
27.42,161.446,0,0,0,0
27.313,164.505,0,0,0,0
26.723,167.135,0,0,0,0
25.542,168.799,0,0,0,0
24.415,170.087,0,0,0,0
23.181,170.409,0,0,0,0
7.51,170.194,0,0,0,0
-18.09,170.087,0,0,0,0
-24.047,168.96,0,0,0,0
-27.482,167.672,0,0,0,0
-30.433,165.901,0,0,0,0
-32.848,162.681,0,0,0,0
-34.136,160.158,0,0,0,0
-34.244,158.709,0,0,0,0
-32.634,143.253,0,0,0,0
-32.419,129.192,0,0,0,0
-32.473,118.351,0,0,0,0
-32.526,105.095,0,0,0,0
-32.634,94.737,0,0,0,0
-32.365,47.832,0,0,0,0
-32.862,-9.539,0,0,0,0
-32.647,-48.072,0,0,0,0
-32.432,-89.611,0,0,0,0
-33.506,-163.243,0,0,0,0
-32.996,-165.792,0,0,0,0
-32.003,-167.992,0,0,0,0
-29.883,-169.629,0,0,0,0
-24.946,-169.709,0,0,0,0
-13.944,-167.455,0,0,0,0
-10.295,-167.348,0,0,0,0
2.478,-167.67,0,0,0,0
9.777,-167.026,0,0,0,0
18.257,-163.806,0,0,0,0
26.414,-157.688,0,0,0,0
28.99,-155.005,0,0,0,0
30.278,-152.107,0,0,0,0
31.029,-148.779,0,0,0,0
30.6,-145.237,0,0,0,0
28.453,-140.407,0,0,0,0
25.341,-136.758,0,0,0,0
22.765,-133.001,0,0,0,0
19.867,-130.639,0,0,0,0
17.076,-127.849,0,0,0,0
15.037,-126.561,0,0,0,0
12.031,-126.453,0,0,0,0
9.884,-127.527,0,0,0,0
8.274,-129.781,0,0,0,0"""

# Split the data into lines and extract the first three elements from each line
lines = data.strip().split('\n')
points = [list(map(float, line.split(',')[:3])) for line in lines]

# Convert the list of points to a NumPy array
points_array = np.array(points)

# Create an Open3D PointCloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_array)

# Save the PointCloud as a PCD file
o3d.io.write_point_cloud("groundtruth.pcd", pcd)

print("PCD file 'output.pcd' has been created using Open3D.")


