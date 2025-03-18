import numpy as np
import genesis as gs

########################## 初始化 ##########################
gs.init(backend=gs.gpu)

########################## 创建场景 ##########################
scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=0.01,#dt：模拟的时间步长，dt 越小，仿真精度越高
        gravity=(0, 0, -10.0),#重力加速度在 x、y、z 三个方向上的分量。这里表示只有在 z 方向上向下的重力加速度（10m/s²）
    ),    
    show_viewer=True,#是否启动场景的可视化窗口
    viewer_options = gs.options.ViewerOptions(
        res = (1280, 960),#窗口的宽度和高度,如果res为None,会创建一个4:3窗口,高度为显示器一半
        camera_pos    = (3, -1, 1.5),
        camera_lookat = (0.0, 0.0, 0.5), 
        camera_fov    = 30,
        max_FPS       = 60,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True, # 显示原点坐标系
        world_frame_size = 1.0, # 坐标系长度(米)
        show_link_frame  = False, # 不显示实体链接坐标系 
        show_cameras     = False, # 显示相机网格和视锥。会比较妨碍观察
        plane_reflection = True, # 开启平面反射
        ambient_light    = (0.1, 0.1, 0.1), # 环境光
    ),
    renderer = gs.renderers.Rasterizer(), # 使用光栅化渲染器.Genesis提供两种渲染器:Rasterizer和RayTracer
    #Rasterizer：光栅，速度快;RayTracer：光线追踪，渲染效果更真实，但速度较慢    
)

########################## 创建实体 ##########################
# 添加地面
scene.add_entity(gs.morphs.Plane())

# 添加目标立方体
cube = scene.add_entity(
    gs.morphs.Box(
        size = (0.04, 0.04, 0.04),
        pos  = (0.65, 0.0, 0.02),
    )
)

# 添加Franka机械臂
franka = scene.add_entity(
    gs.morphs.MJCF(
        file='xml/franka_emika_panda/panda.xml',
        fixed=True,
        pos = (0, 0, 0),#初始位置
        euler = (0, 0, 0), # scipy外旋x-y-z,单位度。朝向
        # quat = (1.0, 0.0, 0.0, 0.0), # w-x-y-z四元数
        scale = 1.0,#缩放),
)

########################## 构建场景 ##########################
scene.build()

########################## 获取控制索引 ##########################

# 定义关节索引
motors_dof = np.arange(7)     # 机械臂关节
fingers_dof = np.arange(7, 9) # 夹爪关节

# 获取末端执行器链接
end_effector = franka.get_link('hand')

########################## 设置控制器参数 ##########################

# 注意：以下值是为实现Franka最佳行为而调整的。
# 有时高质量的URDF或XML文件也会提供这些参数，并会被解析。
franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]), 
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
)

########################## 末端位姿控制 ##########################

# 用IK求解预抓取位姿的关节角度
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
qpos[-2:] = 0.04  # 夹爪打开
# 规划运动路径
path = franka.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2秒时长
)
# 执行规划路径
for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()
# 等待到达最后一个路径点
for i in range(100):
    scene.step()
# 向下移动到抓取位置
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.135]),
    quat = np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()
# 夹紧物体
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)
for i in range(100):
    scene.step()
# 抬起物体
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.3]),
    quat = np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()