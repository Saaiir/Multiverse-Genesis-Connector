
############# Soft & Rigid Box falling on ground #############                                                  ok

# import genesis as gs
#
# gs.init(backend=gs.cpu, seed=0, precision='32', logging_level='info')
#
# dt = 1e-2
#
# scene = gs.Scene(sim_options=gs.options.SimOptions(substeps=10, gravity=(0, 0, -9.8)),
#                  viewer_options=gs.options.ViewerOptions(camera_pos=(0.5, -0.25, 0.5), camera_lookat=(0, 0, 0.0), camera_fov=90),
#                  mpm_options=gs.options.MPMOptions(dt=dt, lower_bound=(-1.0, -1.0, -0.2), upper_bound=(1.0, 1.0, 1.0),),
#                  vis_options=gs.options.VisOptions(show_world_frame=False,),)
#
# scene.add_entity(morph=gs.morphs.Plane())
#
# E, nu = 1e5, 0.3
# rho = 1200.
# cube = scene.add_entity(morph=gs.morphs.Box(lower=(-0.1, -0.1, 0.2), upper=(0.1, 0.1, 0.4),),                               # Soft Box
#                         material=gs.materials.MPM.Elastic(E=E, nu=nu, rho=rho, model='neohooken',),)
#
# rigid_box_1 = scene.add_entity(morph=gs.morphs.Box(lower=(-0.4, -0.4, 0.2), upper=(-0.2, -0.2, 0.4)),                       # Rigid Box
#                              material=gs.materials.Rigid(), surface=gs.surfaces.Default(color=(1.0, 0.0, 0.0)),)
# scene.build()
# for step in range(250):
#     scene.step()

########################################################################################################################

############# Rigid box fixed + Soft & Rigid Box Falling #############                                        ok

# import genesis as gs
#
# gs.init(backend=gs.cpu, seed=0, precision='32', logging_level='info')
#
# dt = 1e-2
# scene = gs.Scene(sim_options=gs.options.SimOptions(substeps=10, gravity=(0, 0, -9.8)),
#                  viewer_options=gs.options.ViewerOptions(camera_pos=(0.35, -0.4, 0.5), camera_lookat=(0, 0, 0.3), camera_fov=90),
#                  mpm_options=gs.options.MPMOptions(dt=dt, lower_bound=(-1.0, -1.0, -0.2), upper_bound=(1.0, 1.0, 1.0)),
#                  rigid_options=gs.options.RigidOptions(dt=dt), vis_options=gs.options.VisOptions(show_world_frame=False),)
#
# scene.add_entity(morph=gs.morphs.Plane())
#
# rigid_box_1 = scene.add_entity(morph=gs.morphs.Box(lower=(-0.1, -0.1, 0.0), upper=(0.1, 0.1, 0.2)),                         # Rigid Box
#                              material=gs.materials.Rigid(), surface=gs.surfaces.Default(color=(1.0, 0.0, 0.0)),)
#
# E, nu, rho = 1e5, 0.3, 1200
# soft_cube = scene.add_entity(morph=gs.morphs.Box(lower=(-0.05, -0.05, 0.3), upper=(0.05, 0.05, 0.4)),                       # Soft box
#                              material=gs.materials.MPM.Elastic(E=E, nu=nu, rho=rho, model='neohooken'),
#                              surface=gs.surfaces.Default())
#
# rigid_box_2 = scene.add_entity(morph=gs.morphs.Box(lower=(-0.05, -0.05, 0.6), upper=(0.05, 0.05, 0.7)),                     # Rigid Box
#                              material=gs.materials.Rigid(), surface=gs.surfaces.Default(color=(1.0, 0.0, 0.0)),)
#
# scene.build()
# for steps in range(100):
#     scene.step()

########################################################################################################################
########################## Soft Box falling on rigid box within mpm boundary ##########################       ok

# import genesis as gs
#
# gs.init(backend=gs.cpu, seed=0, precision='32', logging_level='info')
#
# dt = 1e-2
#
# scene = gs.Scene(sim_options=gs.options.SimOptions(substeps=6, gravity=(0, 0, -9.8)),
#                  viewer_options=gs.options.ViewerOptions(camera_pos=(1.2, -0.3, 0.9), camera_lookat=(0, 0, 0.4), camera_fov=90),
#                  mpm_options=gs.options.MPMOptions(dt=dt, lower_bound=(-0.7, -0.7, -0.2), upper_bound=(0.7, 0.7, 1.0),),
#                  vis_options=gs.options.VisOptions(visualize_mpm_boundary = True, show_world_frame=False,),)
#
# scene.add_entity(morph=gs.morphs.Plane())
#
# box_morph = gs.morphs.Box(size=(0.4, 0.4, 0.4), pos=(0, 0, 0), collision=True, fixed=True)    # creating a box using genesis APIs
# box_entity = scene.add_entity(box_morph)                                                      # adding box entity to scene
# box_entity.name = "box"
#
# E, nu = 1e5, 0.1
# rho = 1200.
# cube= scene.add_entity(morph=gs.morphs.Box(lower=(-0.05, -0.05, 0.8),upper=(0.05, 0.05, 0.9),),                             # Soft box < base box
#                         material=gs.materials.MPM.Elastic(E=E, nu=nu, rho=rho, model='neohooken',),
#                         surface=gs.surfaces.Default(color=(0.8, 0.1, 0.1),))
# # cuboid = scene.add_entity(morph=gs.morphs.Box(lower=(-0.3, -0.3, 0.4),upper=(0.3, 0.3, 0.6),),
# #                         material=gs.materials.MPM.Elastic(E=E, nu=nu, rho=rho, model='neohooken',),                       # Soft box > base box
# #                         surface=gs.surfaces.Default(color=(0.8, 0.1, 0.1),))
# scene.build()
#
# for step in range(600):
#     scene.step()

########################################################################################################################

######################### Soft Box falling on soft box within mpm boundary ##########################             ok

import genesis as gs

gs.init(backend=gs.cpu, seed=0, precision='32', logging_level='info')

dt = 1e-2

scene = gs.Scene(sim_options=gs.options.SimOptions(substeps=5, gravity=(0, 0, -9.8)),
                 viewer_options=gs.options.ViewerOptions(camera_pos=(1.0, -0.7, 1.0), camera_lookat=(0, 0, 0.4), camera_fov=90),
                 rigid_options=gs.options.RigidOptions(dt=dt, gravity=(0, 0, -9.8), enable_collision=True, enable_self_collision=False,),
                 mpm_options=gs.options.MPMOptions(dt=dt, lower_bound=(-0.6, -0.6, -0.5), upper_bound=(0.6, 0.6, 0.8), gravity=(0, 0, -9.8), enable_CPIC=True),
                 vis_options=gs.options.VisOptions(visualize_mpm_boundary = True, show_world_frame=False),)

scene.add_entity(morph=gs.morphs.Plane())

ground_box = scene.add_entity(morph=gs.morphs.Box(lower=(0.0, 0.0, 0), upper=(0.2, 0.2, 0.2),),
                              material=gs.materials.MPM.Elastic(E=1e5, nu=0.1, rho=3000, model='neohooken'),
                              surface=gs.surfaces.Default(color=(0.8, 0.1, 0.1)),)

soft_cube = scene.add_entity(morph=gs.morphs.Box(lower=(-0.1, -0.1, 0.5), upper=(0.1, 0.1, 0.7)),
                             material=gs.materials.MPM.Elastic(E=1e5, nu=0.3, rho=3000, model='neohooken'),
                             surface=gs.surfaces.Default(),)

scene.build()
for steps in range(300):
    scene.step()