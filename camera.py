import os
import cv2
import numpy as np
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
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
cube = scene.add_entity(
    gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(0.65, 0.0, 0.02))
)

########################## add single camera ##########################
# 只使用一台相机，GUI=True表示OpenCV窗口会实时显示渲染画面
cam = scene.add_camera(
    res=(640, 480),
    pos=(2.0, -1.0, 1.5),
    lookat=(0.0, 0.0, 0.5),
    fov=30,
    GUI=True
)

########################## build ##########################
scene.build()

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)
qpos = np.array([
    -1.0124,  1.5559,  1.3662,
    -1.6878, -1.5799,  1.7757,  1.4602,
     0.04,    0.04
])
franka.set_qpos(qpos)
scene.step()

end_effector = franka.get_link("hand")

# 准备一个逆运动学的初始目标
qpos_ik = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.135]),
    quat=np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos_ik[:-2], motors_dof)

########################## set up saving directory ##########################
save_dir = "/home/ur5/genesis/genesis_miaa/shot"
os.makedirs(save_dir, exist_ok=True)

def capture_images(camera, phase_name, step_index):
    """
    渲染RGB、深度、分割掩码和法线图，并保存到指定目录。
    
    camera     : 单个相机对象
    phase_name : 阶段名称(如 "hold", "grasp", "lift")
    step_index : 当前步数, 用于文件命名
    """
    # 渲染相机图像
    rgb, depth, segmentation, normal = camera.render(
        rgb=True, depth=True, segmentation=True, normal=True
    )

    # 1) RGB 图像
    #   cam.render 返回的是 RGB 通道顺序，
    #   OpenCV 的 imwrite 需要 BGR，因此要转换
    bgr_img = rgb[:, :, ::-1]
    rgb_filename = os.path.join(save_dir, f"{phase_name}_{step_index:04d}_rgb.png")
    cv2.imwrite(rgb_filename, bgr_img)

    # 2) 深度图
    #   depth 是一个 (H, W) 的 float32 数组，如果直接保存成 8-bit png，需要先做归一化
    #   在此简单做一个 min-max 的归一化示例
    depth_min, depth_max = depth.min(), depth.max()
    if depth_max > depth_min:  # 避免除 0
        depth_norm = (depth - depth_min) / (depth_max - depth_min)  # 0~1
    else:
        depth_norm = depth - depth_min  # 全 0

    depth_8u = (depth_norm * 255).astype(np.uint8)
    depth_filename = os.path.join(save_dir, f"{phase_name}_{step_index:04d}_depth.png")
    cv2.imwrite(depth_filename, depth_8u)

    # 3) 分割掩码
    #   segmentation 是 (H, W) 整数数组，表示物体 ID
    #   如果仅保存 8-bit PNG，可以 mod 256 或直接转 uint8
    seg_8u = (segmentation % 256).astype(np.uint8)
    seg_filename = os.path.join(save_dir, f"{phase_name}_{step_index:04d}_seg.png")
    cv2.imwrite(seg_filename, seg_8u)

    # 4) 法线图
    #   normal 是 (H, W, 3) 浮点数组，分量范围通常在 -1 ~ 1 之间
    #   将其映射到 0~255 再转 uint8
    normal_255 = (normal * 127.5 + 127.5).astype(np.uint8)  # [-1,1] -> [0,255]
    normal_filename = os.path.join(save_dir, f"{phase_name}_{step_index:04d}_normal.png")
    cv2.imwrite(normal_filename, normal_255)

########################## hold ##########################
phase_name = "hold"
for i in range(100):
    print(f"hold {i}")
    scene.step()
    # 每隔 50 步拍一次图
    if i % 50 == 0:
        capture_images(cam, phase_name, i)

########################## grasp ##########################
phase_name = "grasp"
finder_pos = -0.0
for i in range(100):
    print(f"grasp {i}")
    franka.control_dofs_position(qpos_ik[:-2], motors_dof)
    franka.control_dofs_position(np.array([finder_pos, finder_pos]), fingers_dof)
    scene.step()
    if i % 50 == 0:
        capture_images(cam, phase_name, i)

########################## lift ##########################
phase_name = "lift"
qpos_lift = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.3]),
    quat=np.array([0, 1, 0, 0]),
)
for i in range(200):
    print(f"lift {i}")
    franka.control_dofs_position(qpos_lift[:-2], motors_dof)
    franka.control_dofs_position(np.array([finder_pos, finder_pos]), fingers_dof)
    scene.step()
    if i % 50 == 0:
        capture_images(cam, phase_name, i)
