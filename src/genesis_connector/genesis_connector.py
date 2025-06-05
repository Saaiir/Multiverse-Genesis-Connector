import genesis as gs
import torch
import math
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

class FrankaRobot:
    def __init__(self, scene, franka, mjcf_obj):
        self.scene = scene
        self.entity = franka
        self.mjcf_obj = mjcf_obj

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

    def get_all_joint_positions(self): #returing fixed joints as a list of float e.g [1.0, 2.0, 3.0], revolute as float (in degrees) and prismatic as float (in meters)
        joint_names = self.find_all_joint_names()
        all_positions = {}
        qpos_index = 0

        for joint in self.entity.joints:
            joint_name = joint.name
            joint_type = joint.type.name.lower()

            if joint_type == 'fixed':
                position = joint.get_pos()
                all_positions[joint_name] = position.tolist()
                #print(f"Joint name: {joint_name}, Joint type: {joint_type}, Static position: {position.tolist()} meters")

            elif joint_type == 'revolute':
                joint_value = self.entity.get_qpos([qpos_index])
                joint_value = math.degrees(joint_value.item())
                all_positions[joint_name] = joint_value
                qpos_index += 1
                #print(f"Joint name: {joint_name}, Joint type: {joint_type}, Angle: {joint_value} degrees")

            elif joint_type == 'prismatic':
                joint_value = self.entity.get_qpos([qpos_index])
                joint_value = joint_value.item()
                all_positions[joint_name] = joint_value
                qpos_index += 1
                #(f"Joint name: {joint_name}, Joint type: {joint_type}, Position: {joint_value} meters")

            elif joint_type == "free":
                #print(f"First joint name: {joint_name}, Joint_type 'free' detected — no position extracted.")
                joint_position = None

            else:
                raise ValueError(f"Unknown joint type '{joint_type}' for joint '{joint_name}'.")

        return all_positions

    def set_joint_position(self, joint_name, joint_position):
        for joint in self.entity.joints:
            if joint.name == joint_name:                                                ###### ok
                joint_type = joint.type.name.lower()
                if joint_type == 'fixed':
                    print("Cannot change fixed joint position")
                    return
                if joint_type == 'revolute':
                    position_in_radians = math.radians(joint_position)
                    # Joint position index is required to set qpos value (genesis documentation)
                    self.entity.set_qpos([position_in_radians], [joint.q_idx_local]) # set_qpos() expects two lists (genesis documentation) therefore setting the value 5.0 at index (q_idx_local --- genesis doc), both in list format using sq brackets
                    return
                elif joint_type == 'prismatic':
                    self.entity.set_qpos([joint_position], [joint.q_idx_local])
                    return
        raise ValueError(f"Joint {joint_name} not found!")

    def get_joint_position(self, joint_name): #
        positions = self.get_all_joint_positions()                  ### Getting in degrees
        for joint in self.entity.joints:
            if joint.name == joint_name:
                checked_name_type = joint.type.name.lower()
                checked_name_position = positions[joint_name]
                return checked_name_type, checked_name_position           #returning in degrees
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

    def get_joint_velocity(self, joint_name):
        for joint in self.entity.joints:
            if joint.name == joint_name:
                if joint.type.name.lower() == 'fixed':
                    return None
                dof_velocities = self.entity.get_dofs_velocity()                # extracting all joints velocities here at first --> return type is torch.Tensor (1-element tensor) (genesis doc)
                current_joint_index = joint.dof_idx_local                       # using dof_idx_local to extract the index of specific joint (genesis doc)
                current_joint_velocity = dof_velocities[current_joint_index]    # extracting specific joint value from the tensor
                return current_joint_velocity.item()                            # .item changes the type from 1-element tensor to scalar value
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
        for joint in self.entity.joints:
            if joint.name == joint_name:
                if joint.type.name.lower() == 'fixed':
                    return None
                dof_torques = self.entity.get_dofs_force()  # Gets all joint torques
                current_joint_index = joint.dof_idx_local
                current_joint_torque = dof_torques[current_joint_index]
                return current_joint_torque.item()
        raise ValueError(f"Joint '{joint_name}' not found.")

    def get_model_name_from_mjcf(self):
        # mjcf_obj should be an instance of gs.morphs.MJCF                  # Used Chatgpt here
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
    def __init__(self, franka, obj, scene):
        self.franka = franka
        self.obj = obj
        self.scene = scene  # <- Add scene so we can query all entities

    def get_all_body_names(self):
        names = []

        # Add franka and object names as before
        franka_name = self.franka.get_model_name_from_mjcf()
        if franka_name:
            names.append(franka_name)

        if hasattr(self.obj, 'spawned_objects'):
            names.extend(self.obj.spawned_objects.keys())
        else:
            obj_name = self.obj.get_name()
            if obj_name:
                names.append(obj_name)

        # ✅ Add names from all other entities in the scene
        for entity in self.scene.entities:
            if hasattr(entity, 'name') and entity.name not in names:
                names.append(entity.name)

        return names

def start_simulation(time_step: float, disable_contact: bool) -> Tuple[Any, Any, Any]: #class
    if not getattr(gs, '_initialized', False):
        gs.init(backend=gs.cpu)
    view = gs.options.ViewerOptions(camera_pos=(1.2, 1, 1), camera_lookat=(0, 0, 0.6), camera_fov=90)
    sim = gs.options.SimOptions(dt=time_step)
    scene = gs.Scene(show_viewer=True, viewer_options=view,sim_options=sim)

    object_name = "bread_1"
    object_path = "/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/objects/bread_roll_1/bread_roll_1.xml"
    position = [0.5, 0.7, 0.6]
    orientation = [1.0, 0.0, 0.0, 0.0]

    franka_path = "/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/franka_emika_panda/panda.xml"

    box_morph = gs.morphs.Box(size=(0.1, 0.1, 0.1), pos=(0.5, 0.5, 0.5)) # creating a box using genesis APIs
    box_entity = scene.add_entity(box_morph) # adding box entity to scene
    box_entity.name = "box"

    ### NOTE: Whenever you add primitives to the scene, always set a .name. In Genesis, primitive morphs are like Box, Plane, Cylinder
    ###       These do not automatically create named bodies in the same way MJCF models do.
    ###       If you won´t assign, then the object exists in the simulation, but it won’t appear in scene.get_all_body_names()

    plane_entity = scene.add_entity(gs.morphs.Plane()) # adding plane entity to scene
    plane_entity.name = "plane"

    franka_mjcf = gs.morphs.MJCF(file=franka_path)
    franka = scene.add_entity(franka_mjcf)  # creating and saving franka entity in "franka

    obj = scene.add_entity(gs.morphs.MJCF(file=object_path, pos=position, quat=orientation)) # creating and saving object entity in "obj"

    franka_instance = FrankaRobot(scene,franka, franka_mjcf) #creating franka entity
    object_instance = GenesisObject(scene, object_name, obj) #creating object entity

    scene.build()
    sim_wrapper = SimulationWrapper(franka_instance, object_instance, scene)
    return franka_instance, object_instance, sim_wrapper

def main():
    franka, obj, sim_wrapper  = start_simulation(time_step=0.005, disable_contact=False)

    body_names=sim_wrapper .get_all_body_names()
    print("All body names in the simulation are:", body_names)

    object_name = "bread_1"
    object_position = [0.4, 0.7, 0.9]
    obj.set_position(object_name, object_position)  # setting obj position (uncomment it when want to change the position, comment when want to use original set position)

    assert "bread_1" in obj.spawned_objects, "'bread_1' not found in spawned_objects!"
    print("Spawned objects:", obj.spawned_objects.keys()) # we can also use this if we want to explore available spawned objects names

    max_steps = 200
    for i in range(max_steps):
        steps = franka.step()

        all_joint_names = franka.find_all_joint_names()
        for joint_names in all_joint_names:
            print(f"Joint name: {joint_names}")

        joint_name = "joint2"
        new_joint_position = 5
        franka.set_joint_position(joint_name, new_joint_position)
        joint_type, joint_position = franka.get_joint_position(joint_name)
        if joint_type == 'revolute':
            print(f"joint name: {joint_name}, joint type: {joint_type}, joint_position: {joint_position} degrees")
        else:
            print(f"joint name: {joint_name}, joint type: {joint_type}, joint_position: {joint_position} meters")

        joint_name = "joint6"
        new_joint_velocity = 5
        franka.set_joint_velocity(joint_name,new_joint_velocity)
        joint_velocity = franka.get_joint_velocity(joint_name)
        print(f"joint name: {joint_name}, joint_velocity: {joint_velocity}")

        joint_name = "joint7"
        new_joint_torque =  2
        franka.set_joint_torque(joint_name, new_joint_torque)
        joint_torque = franka.get_joint_torque(joint_name)
        print(f"joint name: {joint_name}, joint_torque: {joint_torque}")

        get_obj_position = obj.get_position(object_name) # getting obj position
        print(f"{object_name}: position: {get_obj_position}")

if __name__ == "__main__":
    main()
