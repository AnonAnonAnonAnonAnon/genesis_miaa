# import numpy as np
# import genesis as gs

# ########################## 初始化 ##########################
# gs.init(backend=gs.gpu)

# ########################## 创建场景 ##########################
# scene = gs.Scene(
#     sim_options=gs.options.SimOptions(
#         dt=0.01,#dt：模拟的时间步长，dt 越小，仿真精度越高
#         gravity=(0, 0, -10.0),#重力加速度在 x、y、z 三个方向上的分量。这里表示只有在 z 方向上向下的重力加速度（10m/s²）
#     ),    
#     show_viewer=True,#是否启动场景的可视化窗口
#     viewer_options = gs.options.ViewerOptions(
#         res = (1280, 960),#窗口的宽度和高度,如果res为None,会创建一个4:3窗口,高度为显示器一半
#         camera_pos    = (3, -1, 1.5),
#         camera_lookat = (0.0, 0.0, 0.5), 
#         camera_fov    = 30,
#         max_FPS       = 60,
#     ),
#     vis_options = gs.options.VisOptions(
#         show_world_frame = True, # 显示原点坐标系
#         world_frame_size = 1.0, # 坐标系长度(米)
#         show_link_frame  = False, # 不显示实体链接坐标系 
#         show_cameras     = False, # 显示相机网格和视锥。会比较妨碍观察
#         plane_reflection = True, # 开启平面反射
#         ambient_light    = (0.1, 0.1, 0.1), # 环境光
#     ),
#     renderer = gs.renderers.Rasterizer(), # 使用光栅化渲染器.Genesis提供两种渲染器:Rasterizer和RayTracer
#     #Rasterizer：光栅，速度快;RayTracer：光线追踪，渲染效果更真实，但速度较慢    
# )

# ########################## 创建实体 ##########################
# # 添加地面
# scene.add_entity(gs.morphs.Plane())

# # 添加目标立方体
# cube = scene.add_entity(
#     gs.morphs.Box(
#         size = (0.04, 0.04, 0.04),
#         pos  = (0.65, 0.0, 0.02),
#     )
# )

# # 添加Franka机械臂
# franka = scene.add_entity(
#     gs.morphs.MJCF(
#         file='xml/franka_emika_panda/panda.xml',
#         pos = (0, 0, 0),#初始位置
#         euler = (0, 0, 0), # scipy外旋x-y-z,单位度。朝向
#         # quat = (1.0, 0.0, 0.0, 0.0), # w-x-y-z四元数
#         scale = 1.0,#缩放),
#     )
# )
# ########################## 构建场景 ##########################
# scene.build()

# ########################## 获取控制索引 ##########################

# # 定义关节索引
# motors_dof = np.arange(7)     # 机械臂关节
# fingers_dof = np.arange(7, 9) # 夹爪关节

# # 获取末端执行器链接
# end_effector = franka.get_link('hand')

# ########################## 设置控制器参数 ##########################

# # 注意：以下值是为实现Franka最佳行为而调整的。
# # 有时高质量的URDF或XML文件也会提供这些参数，并会被解析。
# franka.set_dofs_kp(
#     np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
# )
# franka.set_dofs_kv(
#     np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]), 
# )
# franka.set_dofs_force_range(
#     np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
#     np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
# )

# ########################## 检查关节位姿是否超出限制 ##########################

# # 定义关节名称（请根据实际 MJCF 中的关节名称调整）
# jnt_names = [
#     'joint1',
#     'joint2',
#     'joint3',
#     'joint4',
#     'joint5',
#     'joint6',
#     'joint7',
#     'finger_joint1',
#     'finger_joint2',
# ]

# # 获取当前所有自由度的关节位置
# qpos_start = franka.get_dofs_position()  # 返回一个包含所有关节当前位置的 tensor
# print("当前整体关节位置：", qpos_start)

# # 遍历各个关节，尝试获取限制信息并比较当前位置
# for i, name in enumerate(jnt_names):
#     joint = franka.get_joint(name)
#     pos = qpos_start[i].item()  # 转为 Python 数值以便比较

#     lower_limit = None
#     upper_limit = None

#     # 尝试第一种方式：通过 dof_info 属性（如果存在）
#     if hasattr(joint, 'dof_info'):
#         info = joint.dof_info[0]  # 假设每个 joint 至少有一个自由度
#         lower_limit = info.get('lower', None)
#         upper_limit = info.get('upper', None)
#     # 尝试第二种方式：通过 dof 属性
#     elif hasattr(joint, 'dof'):
#         lower_limit = joint.dof[0].limit_lower
#         upper_limit = joint.dof[0].limit_upper

#     if lower_limit is not None and upper_limit is not None:
#         if pos < lower_limit or pos > upper_limit:
#             print(f"关节 {name}: 当前位置 {pos} 超出限制 [{lower_limit}, {upper_limit}]")
#         else:
#             print(f"关节 {name}: 当前位置 {pos} 在限制范围内 [{lower_limit}, {upper_limit}]")
#     else:
#         print(f"关节 {name}: 没有获取到限制信息")


# ########################## 末端位姿控制 ##########################

# # 用IK求解预抓取位姿的关节角度
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.25]),
#     quat = np.array([0, 1, 0, 0]),
# )
# qpos[-2:] = 0.04  # 夹爪打开
# # 规划运动路径
# path = franka.plan_path(
#     qpos_goal     = qpos,
#     num_waypoints = 200, # 2秒时长
# )
# # 执行规划路径
# for waypoint in path:
#     franka.control_dofs_position(waypoint)
#     scene.step()
# # 等待到达最后一个路径点
# for i in range(100):
#     scene.step()
# # 向下移动到抓取位置
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.135]),
#     quat = np.array([0, 1, 0, 0]),
# )
# franka.control_dofs_position(qpos[:-2], motors_dof)
# for i in range(100):
#     scene.step()
# # 夹紧物体
# franka.control_dofs_position(qpos[:-2], motors_dof)
# franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)
# for i in range(100):
#     scene.step()
# # 抬起物体
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.3]),
#     quat = np.array([0, 1, 0, 0]),
# )
# franka.control_dofs_position(qpos[:-2], motors_dof)
# for i in range(200):
#     scene.step()


###################################################################################################
#关节控制demo，正常运行

# import numpy as np

# import genesis as gs

# ########################## 初始化 ##########################
# gs.init(backend=gs.gpu)

# ########################## 创建场景 ##########################
# scene = gs.Scene(
#     viewer_options = gs.options.ViewerOptions(
#         camera_pos    = (0, -3.5, 2.5),
#         camera_lookat = (0.0, 0.0, 0.5),
#         camera_fov    = 30,
#         res           = (960, 640),
#         max_FPS       = 60,
#     ),
#     sim_options = gs.options.SimOptions(
#         dt = 0.01,
#     ),
#     show_viewer = True,
# )

# ########################## 实体 ##########################
# plane = scene.add_entity(
#     gs.morphs.Plane(),
# )
# franka = scene.add_entity(
#     gs.morphs.MJCF(
#         file  = 'xml/franka_emika_panda/panda.xml',
#     ),
# )
# ########################## 构建 ##########################
# scene.build()

# jnt_names = [
#     'joint1',
#     'joint2',
#     'joint3',
#     'joint4',
#     'joint5',
#     'joint6',
#     'joint7',
#     'finger_joint1',
#     'finger_joint2',
# ]
# dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

# ############ 可选：设置控制增益 ############
# # 设置位置增益
# franka.set_dofs_kp(
#     kp             = np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
#     dofs_idx_local = dofs_idx,
# )
# # 设置速度增益
# franka.set_dofs_kv(
#     kv             = np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
#     dofs_idx_local = dofs_idx,
# )
# # 设置安全的力范围
# franka.set_dofs_force_range(
#     lower          = np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
#     upper          = np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
#     dofs_idx_local = dofs_idx,
# )
# # 硬重置
# for i in range(150):
#     if i < 50:
#         franka.set_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]), dofs_idx)
#     elif i < 100:
#         franka.set_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]), dofs_idx)
#     else:
#         franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]), dofs_idx)

#     scene.step()

# # PD控制
# for i in range(1250):
#     if i == 0:
#         franka.control_dofs_position(
#             np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]),
#             dofs_idx,
#         )
#     elif i == 250:
#         franka.control_dofs_position(
#             np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]),
#             dofs_idx,
#         )
#     elif i == 500:
#         franka.control_dofs_position(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
#             dofs_idx,
#         )
#     elif i == 750:
#         # 用速度控制第一个自由度，其余的用位置控制
#         franka.control_dofs_position(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])[1:],
#             dofs_idx[1:],
#         )
#         franka.control_dofs_velocity(
#             np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0])[:1],
#             dofs_idx[:1],
#         )
#     elif i == 1000:
#         franka.control_dofs_force(
#             np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
#             dofs_idx,
#         )
#     # 这是根据给定控制命令计算的控制力
#     # 如果使用力控制，它与给定的控制命令相同
#     print('控制力:', franka.get_dofs_control_force(dofs_idx))

#     # 这是自由度实际经历的力
#     print('内部力:', franka.get_dofs_force(dofs_idx))

#     scene.step()


###################################################################################################
#末端控制demo，报错，说超出限制


# import numpy as np
# import genesis as gs

# ########################## 初始化 ##########################
# gs.init(backend=gs.gpu)

# ########################## 创建场景 ##########################
# scene = gs.Scene(
#     viewer_options = gs.options.ViewerOptions(
#         camera_pos    = (3, -1, 1.5),
#         camera_lookat = (0.0, 0.0, 0.5), 
#         camera_fov    = 30,
#         max_FPS       = 60,
#     ),
#     sim_options = gs.options.SimOptions(
#         dt = 0.01,
#     ),
#     show_viewer = True,
# )

# ########################## 创建实体 ##########################
# # 添加地面
# scene.add_entity(gs.morphs.Plane())

# # 添加目标立方体
# cube = scene.add_entity(
#     gs.morphs.Box(
#         size = (0.04, 0.04, 0.04),
#         pos  = (0.65, 0.0, 0.02),
#     )
# )

# # 添加Franka机械臂
# franka = scene.add_entity(
#     gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
# )

# ########################## 构建场景 ##########################
# scene.build()

# # 定义关节索引
# motors_dof = np.arange(7)     # 机械臂关节
# fingers_dof = np.arange(7, 9) # 夹爪关节

# # 设置控制器参数
# # 注意：以下值是为实现Franka最佳行为而调整的。
# # 有时高质量的URDF或XML文件也会提供这些参数，并会被解析。
# franka.set_dofs_kp(
#     np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
# )
# franka.set_dofs_kv(
#     np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]), 
# )
# franka.set_dofs_force_range(
#     np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
#     np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
# )

# # 获取末端执行器链接
# end_effector = franka.get_link('hand')

# # 用IK求解预抓取位姿的关节角度
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.25]),
#     quat = np.array([0, 1, 0, 0]),
# )
# qpos[-2:] = 0.04  # 夹爪打开

# # 规划运动路径
# path = franka.plan_path(
#     qpos_goal     = qpos,
#     num_waypoints = 200, # 2秒时长
# )

# # 执行规划路径
# for waypoint in path:
#     franka.control_dofs_position(waypoint)
#     scene.step()

# # 等待到达最后一个路径点
# for i in range(100):
#     scene.step()

# # 向下移动到抓取位置
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.135]),
#     quat = np.array([0, 1, 0, 0]),
# )
# franka.control_dofs_position(qpos[:-2], motors_dof)
# for i in range(100):
#     scene.step()

# # 夹紧物体
# franka.control_dofs_position(qpos[:-2], motors_dof)
# franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)
# for i in range(100):
#     scene.step()

# # 抬起物体
# qpos = franka.inverse_kinematics(
#     link = end_effector,
#     pos  = np.array([0.65, 0.0, 0.3]),
#     quat = np.array([0, 1, 0, 0]),
# )
# franka.control_dofs_position(qpos[:-2], motors_dof)
# for i in range(200):
#     scene.step()


###################################################################################################
# https://github.com/Genesis-Embodied-AI/Genesis/blob/main/examples/rigid/franka_cube.py

import numpy as np
import time
import genesis as gs

########################## init ##########################
gs.init(backend=gs.gpu, precision="32")
########################## create a scene ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(3, -1, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=30,
        res=(960, 640),
        max_FPS=60,
    ),
    sim_options=gs.options.SimOptions(
        dt=0.01,
    ),
    rigid_options=gs.options.RigidOptions(
        box_box_detection=True,
    ),
    show_viewer=True,
)

########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
)

cube = scene.add_entity(
    gs.morphs.Box(
        size=(0.04, 0.04, 0.04),
        pos=(0.65, 0.0, 0.02),
    )
)
########################## build ##########################
scene.build()

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)
qpos = np.array([-1.0124, 1.5559, 1.3662, -1.6878, -1.5799, 1.7757, 1.4602, 0.04, 0.04])
franka.set_qpos(qpos)
scene.step()

end_effector = franka.get_link("hand")
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.135]),
    quat=np.array([0, 1, 0, 0]),
)

franka.control_dofs_position(qpos[:-2], motors_dof)

# hold
for i in range(100):
    print("hold", i)
    scene.step()

# grasp
finder_pos = -0.0
for i in range(100):
    print("grasp", i)
    franka.control_dofs_position(qpos[:-2], motors_dof)
    franka.control_dofs_position(np.array([finder_pos, finder_pos]), fingers_dof)
    scene.step()

# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.3]),
    quat=np.array([0, 1, 0, 0]),
)
for i in range(200):
    print("lift", i)
    franka.control_dofs_position(qpos[:-2], motors_dof)
    franka.control_dofs_position(np.array([finder_pos, finder_pos]), fingers_dof)
    scene.step()