import genesis as gs
from typing import Any,Tuple

class GenesisObject:
    def __init__(self,scene,object_name, obj):
        self.name = object_name
        self.scene = scene
        self.entity = obj
        self.spawned_objects = {object_name: obj}

    def get_name(self):
        if hasattr(self.entity, 'mjcf') and hasattr(self.entity.mjcf, 'root'):
            return self.entity.mjcf.root.name
        return self.name

    def get_position(self, object_name):
        if object_name in self.spawned_objects:
            pos = self.spawned_objects[object_name].get_pos()
            position = pos.tolist() # Return as list for easy comparison
            print(f"{object_name}: position: {position}")
            return position
        else:
            raise ValueError(f"Object '{object_name}' not found.")

    def set_position(self, object_name, position):
        if object_name in self.spawned_objects:
            self.spawned_objects[object_name].set_pos(position)
            return
        else:
            raise ValueError(f"Object '{object_name}' not found.")

    def get_velocity(self, object_name):
        if object_name in self.spawned_objects:
            velocity = self.entity.get_vel()  # return type is torch.Tensor (3-element tensor) in m/s
            return velocity
        else:
            raise ValueError(f"Object '{object_name}' not found.")

class Robot:
    def __init__(self, scene, robot, mjcf_obj):
        self.scene = scene
        self.entity = robot
        self.mjcf_obj = mjcf_obj
        self._position_call_tracker = set()

    def step(self):
        self.scene.step()

    def get_name(self):
        if hasattr(self.entity, 'mjcf') and hasattr(self.entity.mjcf, 'root'): # Trying to get MJCF root name if exists
            return self.entity.mjcf.root.name
        return type(self.entity).__name__

    def find_all_joint_names(self):
        joint_names=[]
        for joint in self.entity.joints:
            joint_names.append(joint.name)
        return joint_names

    def get_all_joint_positions(self):
        all_positions = {}      #returing fixed joints as a list of float e.g [1.0, 2.0, 3.0], revolute as float (in radians) and prismatic as float (in meters)
        for joint in self.entity.joints:
            joint_name = joint.name
            joint_type = joint.type.name.lower()
            if joint_type == 'fixed':
                position = joint.get_pos()
                all_positions[joint_name] = position.tolist()
            elif joint_type in ['revolute', 'prismatic']:
                qpos_index = joint.q_idx_local                                  # Using the joint's actual qpos index here now
                joint_value = self.entity.get_qpos([qpos_index])
                all_positions[joint_name] = joint_value
            elif joint_type == 'free':
                position = None
                all_positions[joint_name] = position
                continue
            else:
                raise ValueError(f"Unknown joint type '{joint_type}' for joint '{joint_name}'.")
        return all_positions

    def set_joint_position(self, joint_name, joint_position):
        for joint in self.entity.joints:
            if joint.name == joint_name:
                joint_type = joint.type.name.lower()
                if joint_type in ['revolute', 'prismatic']:                       # Joint position index is required to set qpos value (genesis documentation)
                    self.entity.set_qpos([joint_position], [joint.q_idx_local])   # set_qpos() expects two lists (genesis documentation) therefore setting the value (any) at index (q_idx_local --- genesis doc), both in list format using sq brackets
                    return
                else:
                    print("Cannot change {joint_type} joint position")
                    return
        raise ValueError(f"Joint {joint_name} not found!")

    def get_joint_position(self, joint_name): #
        positions = self.get_all_joint_positions()                                  ### Getting in radians

        # if not hasattr(self, "_position_call_tracker"):
        #     self._position_call_tracker = set()
        is_first_call = joint_name not in self._position_call_tracker
        if is_first_call:
            self._position_call_tracker = set()
        else:
            self._position_call_tracker.add(joint_name)

        for joint in self.entity.joints:
            if joint.name == joint_name:
                checked_name_type = joint.type.name.lower()
                checked_name_position = positions[joint_name]
                if checked_name_type in ['fixed', 'free', 'prismatic']:
                    print(f"joint name: {joint_name}, joint type: {checked_name_type}, {'actual' if is_first_call else 'updated'}_joint_positon: {checked_name_position} meters")
                else:
                    print(f"joint name: {joint_name}, joint type: {checked_name_type}, {'actual' if is_first_call else 'updated'}_joint_positon: {checked_name_position} radians")
                return checked_name_type, checked_name_position           #returning in radians
        raise ValueError(f"Joint {joint_name} not found!")

    def set_joint_velocity(self, joint_name, velocity):
        for joint in self.entity.joints:
            if joint.name == joint_name:
                joint_type = joint.type.name.lower()
                if joint_type in ['revolute', 'prismatic']:
                    self.entity.set_dofs_velocity([velocity], [joint.dof_idx_local])
                    return
                else:
                    print(f"Joint '{joint_name}' is of type '{joint_type}' and does not support to set velocity.")
                    return
        raise ValueError(f"Joint '{joint_name}' not found.")

    def get_joint_velocity(self,joint_name):

        if not hasattr(self, "_velocity_call_tracker"):
            self._velocity_call_tracker = set()
        is_first_call = joint_name not in self._velocity_call_tracker
        self._velocity_call_tracker.add(joint_name)

        for joint in self.entity.joints:
            if joint.name == joint_name:
                joint_type = joint.type.name.lower()
                if joint_type == 'fixed':
                    current_joint_velocity = 0
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_velocity: {current_joint_velocity}")
                    return current_joint_velocity
                elif joint_type == 'free':
                    current_joint_velocity = 0
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_velocity: discuss about free joint velocity")
                    return current_joint_velocity
                else:
                    dof_velocities = self.entity.get_dofs_velocity()                # extracting all joints velocities here at first --> return type is torch.Tensor (1-element tensor) (genesis doc)
                    current_joint_index = joint.dof_idx_local                       # using dof_idx_local to extract the index of specific joint (genesis doc)
                    current_joint_velocity = dof_velocities[current_joint_index]    # extracting specific joint value from the tensor
                    # print(current_joint_velocity)
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_velocity: {current_joint_velocity.item()}") # .item changes the type from 1-element tensor to scalar value
                    return current_joint_velocity.item()
        raise ValueError(f"Joint '{joint_name}' not found.")

    def set_joint_torque(self, joint_name, torque):
        for joint in self.entity.joints:
            if joint.name == joint_name:
                joint_type = joint.type.name.lower()
                if joint_type in ['revolute', 'prismatic']:
                    self.entity.control_dofs_force([torque], [joint.dof_idx_local])
                    return
                else:
                    print(f"Joint '{joint_name}' is of type '{joint_type}' and does not support to set torque.")
                    return
        raise ValueError(f"Joint '{joint_name}' not found.")

    def get_joint_torque(self, joint_name):

        if not hasattr(self, "_torque_call_tracker"):
            self._torque_call_tracker = set()
        is_first_call = joint_name not in self._torque_call_tracker
        self._torque_call_tracker.add(joint_name)

        for joint in self.entity.joints:
            if joint.name == joint_name:
                joint_type = joint.type.name.lower()
                if joint_type == 'fixed':
                    current_joint_torque = 0
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_torque: {current_joint_torque}")
                    return current_joint_torque
                elif joint_type == 'free':
                    current_joint_torque = 0
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_torque: discuss about free joint torque")
                    return current_joint_torque
                else:
                    dof_torques = self.entity.get_dofs_force()  # Gets all joint torques
                    current_joint_index = joint.dof_idx_local
                    current_joint_torque = dof_torques[current_joint_index]
                    print(f"joint name: {joint_name}, joint type: {joint_type}, {'actual' if is_first_call else 'updated'}_joint_torque: {current_joint_torque}")  # .item changes the type from 1-element tensor to scalar value
                    return current_joint_torque.item()
        raise ValueError(f"Joint '{joint_name}' not found.")

    def get_model_name_from_mjcf(self):
        # mjcf_obj should be an instance of gs.morphs.MJCF                          # Used Chatgpt here
        try:
            if hasattr(self.mjcf_obj, 'root'):
                # root should be the root XML element
                model_name = self.mjcf_obj.root.name
                if model_name:
                    return model_name
            # fallback
            if hasattr(self.mjcf_obj, 'file'):
                # get filename as fallback model name
                import os
                return os.path.splitext(os.path.basename(self.mjcf_obj.file))[0]
        except Exception as e:
            print(f"Error reading model name: {e}")
        return "UnknownModelName"

class SimulationWrapper:
                                                                                    # Used Chatgpt here
    def __init__(self, robots, obj, scene):
        self.robots = robots
        self.obj = obj
        self.scene = scene

    def get_all_body_names(self):
        names = []
        for robo in self.robots:
            robot_name = robo.get_model_name_from_mjcf()
            if robot_name:
                if robot_name not in names:
                    names.append(robot_name)

        if hasattr(self.obj, 'spawned_objects'):
            for obj_name in self.obj.spawned_objects.keys():
                if obj_name not in names:
                    names.append(obj_name)
        else:
            obj_name = self.obj.get_name()
            if obj_name and obj_name not in names:
                names.append(obj_name)

        for entity in self.scene.entities:
            if hasattr(entity, 'name') and entity.name not in names:
                names.append(entity.name)
        return names

def start_simulation(time_step: float, disable_contact: bool) -> Tuple[Any, Any, Any]: #class
    if not getattr(gs, '_initialized', False):
        gs.init(backend=gs.cpu)
    view = gs.options.ViewerOptions(camera_pos=(1.6, 0, 1.1), camera_lookat=(0, 0, 0.5), camera_fov=90)
    sim = gs.options.SimOptions(dt=time_step)
    scene = gs.Scene(show_viewer=True, viewer_options=view,sim_options=sim)

    ### NOTE: Whenever you add primitives to the scene, always set a .name. In Genesis, primitive morphs are like Box, Plane, Cylinder
    ###       These do not automatically create named bodies in the same way MJCF models do.
    ###       If you won´t assign, then the object exists in the simulation, but it won’t appear in scene.get_all_body_names()

    plane_entity = scene.add_entity(gs.morphs.Plane())                  # adding plane entity to scene
    plane_entity.name = "plane"

    box_morph = gs.morphs.Box(size=(0.1, 0.1, 0.1), pos=(1, 0, 0.5))    # creating a box using genesis APIs
    box_entity = scene.add_entity(box_morph)                            # adding box entity to scene
    box_entity.name = "box"

    object_name = "bread_1"
    object_path = "/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/objects/bread_roll_1/bread_roll_1.xml"
    position = [0.0, 0.0, 0.5]
    orientation = [1.0, 0.0, 0.0, 0.0]
    obj = scene.add_entity(gs.morphs.MJCF(file=object_path, pos=position, quat=orientation))    # creating and saving object entity in "obj"
    object_instance = GenesisObject(scene, object_name, obj)                                    # creating object entity

    franka_path = "/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/franka_emika_panda/panda.xml"
    franka_mjcf = gs.morphs.MJCF(file=franka_path)
    franka = scene.add_entity(franka_mjcf)                                                      # creating and saving franka entity in "franka
    franka_instance = Robot(scene, franka, franka_mjcf)                                         # creating franka entity

    talos_path = "/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/pal_talos/talos.xml"
    talos_mjcf = gs.morphs.MJCF(file=talos_path)
    talos = scene.add_entity(talos_mjcf)
    talos_instance = Robot(scene, talos, talos_mjcf)

    scene.build()
    franka.set_pos([0.0, 0.0, 0.0])
    talos.set_pos([0.0, -1.0, 1.0])

    robots_instances = [franka_instance, talos_instance]
    sim_wrapper = SimulationWrapper(robots_instances, object_instance, scene)

    return robots_instances, object_instance, sim_wrapper

def main():
    robots, obj, sim_wrapper = start_simulation(time_step=0.005, disable_contact=False)

    for i, robot in enumerate(robots):
        joints = robot.find_all_joint_names()
        print(f"Robot {i} all joint names are:", joints)

    body_names = sim_wrapper .get_all_body_names()
    print("All body names in the simulation are:", body_names)

    object_name = "bread_1"
    object_position = [0.0, 1.0, 0.9]
    obj.set_position(object_name, object_position)  # setting obj position (uncomment it when want to change the position, comment when want to use original set position)

    max_steps = 200
    robot_joints_list = []

    for robot in robots:
        joints = robot.find_all_joint_names()
        joints_set = set(joints)                                                            # converting the list to a set
        robot_joints_list.append(joints_set)

    for i in range(max_steps):
        for idx, robot in enumerate(robots):
            robot.step()

            joint_name = "torso_1_joint"
            if joint_name in robot_joints_list[idx]:
                new_joint_position = 1.2                    #(68 degrees) 0.5236 #(30 degrees)
                robot.get_joint_position(joint_name)
                robot.set_joint_position(joint_name, new_joint_position)

            joint_name = "joint1"
            if joint_name in robot_joints_list[idx]:
                new_joint_position =  0.5236                #(30 degrees)
                robot.get_joint_position(joint_name)
                robot.set_joint_position(joint_name, new_joint_position)

            joint_name = "leg_left_1_joint"
            if joint_name in robot_joints_list[idx]:
                new_joint_velocity = 1
                if i == 0:
                    robot.get_joint_velocity(joint_name)
                    robot.set_joint_velocity(joint_name,new_joint_velocity)
                else:
                    robot.set_joint_velocity(joint_name, new_joint_velocity)
                    robot.get_joint_velocity(joint_name)

            joint_name = "joint6"
            if joint_name in robot_joints_list[idx]:
                new_joint_velocity = 2
                if i == 0:
                    robot.get_joint_velocity(joint_name)
                    robot.set_joint_velocity(joint_name, new_joint_velocity)
                else:
                    robot.set_joint_velocity(joint_name, new_joint_velocity)
                    robot.get_joint_velocity(joint_name)

            joint_name = "leg_right_1_joint"
            if joint_name in robot_joints_list[idx]:
                new_joint_torque =  1
                robot.get_joint_torque(joint_name)
                robot.set_joint_torque(joint_name, new_joint_torque)

            joint_name = "joint7"
            if joint_name in robot_joints_list[idx]:
                new_joint_torque = 2
                robot.get_joint_torque(joint_name)
                robot.set_joint_torque(joint_name, new_joint_torque)

        object_name = "bread_1"
        obj.get_position(object_name)

if __name__ == "__main__":
    main()
