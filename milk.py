import argparse

import genesis as gs


import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", default=True)
    args = parser.parse_args()

    ########################## init ##########################
    gs.init(seed=0, precision="32", logging_level="debug")

    ########################## create a scene ##########################

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=2e-3,
            substeps=10,
        ),
        vis_options=gs.options.VisOptions(
            visualize_sph_boundary=True,
            visualize_mpm_boundary=True,
        ),
        mpm_options=gs.options.MPMOptions(
            lower_bound=(-1.0, -1.0, -0.1),
            upper_bound=(1.0, 1.0, 1.5),
            grid_density=64,
            # particle_size = 0.005,
        ),
        sph_options=gs.options.SPHOptions(
            lower_bound=(-1.03, -1.03, -0.08),
            upper_bound=(1.03, 1.03, 1.5),
            particle_size = 0.01,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -2, 1.2),
            camera_lookat=(0.0, 0.0, 0.1),
            camera_fov=30,
            max_FPS=60,
        ),
        show_viewer=True,
    )

    ########################## entities ##########################
    plane = scene.add_entity(
        morph=gs.morphs.Plane(),
    )

    
    water = scene.add_entity(
        material=gs.materials.SPH.Liquid(
            rho=1000.0,
            stiffness=50000.0,
            exponent=7.0,
            mu=0.005,
            gamma=0.3,
            sampler="pbs",
        ),
        morph=gs.morphs.Box(
            pos=(0.0, 0.0, 0.2),
            size=(0.25, 0.25, 0.2),
        ),
        surface=gs.surfaces.Default(
            # color=(0.2, 0.6, 1.0, 1.0),
            color=(1, 1, 1 , 0.9),
            # vis_mode="visual",
        ),
    )

    # water = scene.add_entity(
    #     material=gs.materials.MPM.Liquid(
    #         E=1e6,
    #         nu=0.2,
    #         rho=1000.0,
    #         lam=None,
    #         mu=None,
    #         viscous=False,
    #         sampler="pbs",
    #     ),
    #     morph=gs.morphs.Box(
    #         pos=(0.0, 0.0, 0.25),
    #         size=(0.2, 0.2, 0.2),
    #     ),
    #     surface=gs.surfaces.Default(
    #         # color=(0.2, 0.6, 1.0, 1.0),
    #         color=(1, 1, 1 , 0.5),
    #         vis_mode="particle",
    #     ),
    # )

    num = 2
    for i in range(num) : 
        pos_ran = 0.12
        pos_rand_x = np.random.uniform(-pos_ran, pos_ran)
        pos_rand_y = np.random.uniform(-pos_ran, pos_ran)

        rot_ran = 90
        rot_rand_x = np.random.uniform(0, rot_ran)
        rot_rand_y = np.random.uniform(0, rot_ran)
        rot_rand_z = np.random.uniform(0, rot_ran)

        
 
        duck = scene.add_entity(
            material=gs.materials.MPM.Elastic(rho=200),
            morph=gs.morphs.Mesh(
                file="meshes/duck.obj",
                pos=(0.0 + pos_rand_x, 0.0 + pos_rand_y, 0.35),
                euler = (rot_rand_x, rot_rand_y, rot_rand_z),
                scale=(0.02, 0.02, 0.005),
            ),
            surface=gs.surfaces.Default(
                color=(0.9, 0.8, 0.2, 1.0),
                vis_mode="visual",
            ),
        )

    bowl = scene.add_entity(
        morph=gs.morphs.Mesh(
                file="meshes/demo_bowl.obj",
                pos=(0.0, 0.0, 0),
                euler=(0, 0.0, 0.0),
                scale=0.5,
                convexify = False,
                # decimate = False,
                decimate_face_num = 10000,
                fixed = True,
                # decompose_nonconvex = False
            ),

        material=gs.materials.Rigid(
            rho=300,
            friction=0.2,
            needs_coup=True,
            coup_restitution = 0.0001,
            sdf_cell_size=0.000001
        ),
        surface=gs.surfaces.Default(
            color    = (150/255, 180/255, 230/255),
            vis_mode = 'visual',
        )
    )

    
    ########################## build ##########################
    scene.build()

    for i in range(30000):
        scene.step()


if __name__ == "__main__":
    main()
