import genesis as gs
gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    #gs.morphs.MJCF(file="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/objects/bread_roll_1/bread_roll_1.xml"))
    gs.morphs.MJCF(file="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/shadow_hand/scene_left.xml"))


scene.build()

for i in range(200):
    scene.step()