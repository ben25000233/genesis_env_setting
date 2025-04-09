import genesis as gs
import numpy as np
from genesis.utils.geom import trans_quat_to_T, xyz_to_quat, quat_to_T
from scipy.spatial.transform import Rotation as R
# gs.init(backend=gs.gpu)
gs.init()

scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt       = 2e-3,
        substeps = 10,
    ),
    # rigid_options = gs.options.RigidOptions(
    #     dt       = 1e-2,
    # ),
    sph_options=gs.options.SPHOptions(
        lower_bound   = (-2, -2, 0),
        upper_bound   = (2, 2, 2),
        particle_size = 0.005,
        # particle_size = 0.005,
    ),
    mpm_options=gs.options.MPMOptions(
        lower_bound   = (-2, -2, 0),
        upper_bound   = (2, 2, 2),
        # grid_density = 64,
    ),
    vis_options=gs.options.VisOptions(
        visualize_sph_boundary = True,
    ),
    show_viewer = True,
)

plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    gs.morphs.MJCF(file='./meshes/new_spoon_franka/panda.xml',convexify=False),
    surface=gs.surfaces.Default(
        vis_mode = 'collision',
    )
)
    

# table = scene.add_entity(
#     gs.morphs.Box(size=(1.0, 1.5, 0.046), pos=(0.8, 0, 0.023), fixed=True)
# )


# liquid = scene.add_entity(
#     material=gs.materials.SPH.Liquid(
#     ),
#     morph=gs.morphs.Box(
#         lower = (0.56, -0.16 , 0.05),
#         upper = (0.62, -0.1 , 0.15)
        
#     ),
#     surface=gs.surfaces.Default(
#         color    = (0.4, 0.8, 1.0),
#         vis_mode = 'particle',
#     ),
# )

water = scene.add_entity(
    material=gs.materials.SPH.Liquid(
        rho=1000.0,
        stiffness=50000.0,
        exponent=7.0,
        mu=0.005,
        gamma=0.03,
        sampler="pbs",
    ),
    morph=gs.morphs.Box(
        lower = (0.56, -0.16 , 0.15),
        upper = (0.62, -0.1 , 0.25)
    ),
    surface=gs.surfaces.Default(
        # color=(0.2, 0.6, 1.0, 1.0),
        color=(1, 1, 1 , 0.5),
        vis_mode="particle",
    ),
)

# mat_bowl = gs.materials.Rigid(
#         needs_coup=True,
#         coup_softness=0.001,
#         sdf_cell_size = 0.0005,
#         rho = 2000
#     )

obj_bowl = scene.add_entity(
        morph=gs.morphs.Mesh(
                file="meshes/s_bowl.obj",
                pos = (0.59, -0.11 , 0.05),
                euler=(0, 0.0, 0.0),
                scale=(1/6, 1/6, 1/6),
                convexify = False,
                # decimate = False,
                decimate_face_num = 10000,
                fixed = True,
                # decompose_nonconvex = False
            ),

        material=gs.materials.Rigid(
            rho=300,
            friction=0.5,
            needs_coup=True,
            coup_restitution = 0.0001,
            sdf_cell_size=0.000001
        ),
        surface=gs.surfaces.Default(
            color    = (0.4, 0.5, 0.4),
            vis_mode = 'vision',
        )
    )

cam = scene.add_camera(
            res=(1980, 1080),
            pos=(2, 0, 1.5),    
            lookat=(0.0, -0.0, 0.06),
            fov=30,
            GUI=False,
        )

#hand = franka.get_link('panda_hand')
#end_effector = franka.get_link('left_finger')

scene.build()

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

cam.start_recording()

data = np.load("./sample_trial/col_03/joint_states.npy")

#data_quat = np.load("/media/hcis-s21/team/chiyi/dataset_1_18/coh_01/ee_pose_qua.npy")

initial_pose = data[0]
franka.control_dofs_position(np.hstack((initial_pose, np.array([0.04, 0.04]))))

for i in range(200):
    cam.render()
    scene.step()

data = np.hstack((data, np.full((231, 2), 0.04)))
for waypoint in data:
    # franka.control_dofs_position(waypoint)
    
    for _ in range(15):
        scene.step()
        cam.render()
    
cam.stop_recording(save_to_filename='video3.mp4', fps=60)