import numpy as np
import tf.transformations as tfs

to_rad = np.math.pi / 180

# mat is lidar to cam
mat = np.loadtxt('extrin.txt', skiprows=1)

# print(mat)

# turn to gazebo_cam to lidar
el = to_rad * np.array([-90, 0, -90])
Rcam = tfs.euler_matrix(el[0], el[1], el[2])
# mat = np.linalg.inv(mat)
mat = np.matmul(Rcam, mat)

rpy = np.array(tfs.euler_from_matrix(mat[:3, :3]))

print('rpy(rad): {}'.format(rpy))
print('rpy(deg): {}'.format(rpy * 180 / np.math.pi))
print(mat)
print(mat[:3, 3])
