import open3d as o3d
import numpy as np
import copy


ptCloud = o3d.io.read_point_cloud('/home/space/work/1_DucoMould/tail_points/gcode_wall_5_1.pcd')

ptCloud_r = copy.deepcopy(ptCloud)

R = ptCloud.get_rotation_matrix_from_xyz((0,0,np.pi))

ptCloud_r.rotate(R, center=(0,0,0))

# o3d.visualization.draw_geometries([ptCloud, ptCloud_r])

o3d.io.write_point_cloud('/home/space/work/1_DucoMould/tail_points/gcode_wall_5_2.pcd',ptCloud_r)








# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
# mesh_r = copy.deepcopy(mesh)
# R = mesh.get_rotation_matrix_from_xyz((0,0,np.pi/2))#欧拉角转变换矩阵
# '''
# 函数rotate的第二个参数center默认为True。
# 这表示对象在旋转之前首先居中，然后移动到先前的中心。
# 如果设置为False，则几何图像将直接围绕坐标中心旋转。
# 这意味着网格中心可以在旋转之后改变。
# '''
# mesh_r.rotate(R, center=(0,0,0))
# mesh_r.paint_uniform_color([1, 0.75, 0])#旋转后的渲染为金色
# o3d.visualization.draw_geometries([mesh, mesh_r])
