import argparse
import torch
import math
import genesis as gs

def get_inputs ():
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", help="/Path to the XML", default="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/franka_fr3/fr3.xml")
    #parser.add_argument("--input", help="/Path to the XML", default="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/anybotics_anymal_c/anymal_c.xml")
    #parser.add_argument("--input", help="/Path to the XML", default="//home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/agilex_piper/piper.xml")
    parser.add_argument("--time_step", type=float, help="Time step in seconds", default=0.005)

    args = parser.parse_args()
    file_path = args.input
    sim_time = args.time_step

    return file_path, sim_time

class Entity:
    def __init__(self,name):
        self.name = name

    def get_pose(self):
        pass

    def get_velocity(self):
        pass

class Robot:
    def __init__(self,name,genesis_entity):
        self.name = name
        self.entity = genesis_entity

    def get_pose(self):
        position=self.entity.get_pos()
        orientation=self.entity.get_quat()
        pose=torch.cat((position, orientation), dim=0)
        print(f"Robot pose: {pose}")
        return pose

    def get_velocity(self):
        robot_velocity = self.entity.get_vel()
        print(f"Robot velocity: {robot_velocity}")
        return robot_velocity

    def find_all_link_names(self):
        link_names=[]
        for link in self.entity.links:
            link_names.append(link.name)
        return link_names

    def get_single_link_pos(self):

        link_names = self.find_all_link_names()
        first_link_name = link_names[1]
        index_of_first_link = link_names.index(first_link_name)

        all_positions = self.entity.get_links_pos()  # (num_links, 3)
        all_orientations = self.entity.get_links_quat()  # (num_links, 4)
        position = all_positions[index_of_first_link]
        orientation = all_orientations[index_of_first_link]
        pose = torch.cat((position, orientation), dim=0)
        print(f"First link name: {first_link_name} , pose: {pose}")
        return pose

    def get_all_link_poses(self):
        link_poses = {}
        all_positions = self.entity.get_links_pos()
        all_orientations = self.entity.get_links_quat()
        for index, link in enumerate(self.entity.links):
            position = all_positions[index]
            orientation = all_orientations[index]
            pose = torch.cat((position, orientation), dim=0)
            link_poses[link.name] = pose
        for name in link_poses:
            print(name, link_poses[name])
        return link_poses
    
    def find_all_joint_names(self):
        joint_names=[]
        for joint in self.entity.joints:
            joint_names.append(joint.name)
        #print(joint_names)
        return joint_names

    def get_all_joint_positions(self):
        joint_names = self.find_all_joint_names()
        all_positions = {}
        qpos_index = 0

        for joint in self.entity.joints:
            joint_name = joint.name
            joint_type = joint.type.name.lower()

            if joint_type == 'fixed':
                position = joint.get_pos()
                all_positions[joint_name] = position.tolist()
                print(f"Joint name: {joint_name}, Joint type: {joint_type}, Static position: {position.tolist()} meters")

            elif joint_type == 'revolute':
                joint_value = self.entity.get_qpos([qpos_index])
                joint_value = math.degrees(joint_value.item())
                all_positions[joint_name] = joint_value
                qpos_index += 1
                print(f"Joint name: {joint_name}, Joint type: {joint_type}, Angle: {joint_value} degrees")

            elif joint_type == 'prismatic':
                joint_value = self.entity.get_qpos([qpos_index])
                joint_value = joint_value.item()
                all_positions[joint_name] = joint_value
                qpos_index += 1
                print(f"Joint name: {joint_name}, Joint type: {joint_type}, Position: {joint_value} meters")

            elif joint_type == "free":
                print(f"First joint name: {joint_name}, Joint_type 'free' detected — no position extracted.")
                joint_position = None

            else:
                raise ValueError(f"Unknown joint type '{joint_type}' for joint '{joint_name}'.")

        return all_positions

    def get_first_joint_position(self):
        joint_names = self.find_all_joint_names()
        first_joint_name = joint_names[0]
        joint_position =[]

        for joint in self.entity.joints:
            if joint.name == first_joint_name:
                joint_type = joint.type.name.lower()

                if joint_type == 'fixed':
                    position = joint.get_pos()
                    joint_position = position
                    print(f"Joint name: {first_joint_name}, Joint type: {joint_type}, Static position: {position.tolist()} meters")

                elif joint_type == 'revolute':
                    position = self.entity.get_qpos([0])
                    joint_value = math.degrees(position.item())
                    joint_position = joint_value
                    print(f"First joint name: {first_joint_name}, Joint_type '{joint_type}', Angle: {joint_value} degree")

                elif joint_type == "prismatic":
                    position = self.entity.get_qpos([0])
                    joint_value = position.item()
                    joint_position = joint_value
                    print(f"First joint name: {first_joint_name}, Joint_type '{joint_type}', Position: {joint_value} meters")

                elif joint_type == "free":
                    print(f"First joint name: {first_joint_name}, Joint_type 'free' detected — no position extracted.")
                    joint_position = None

                else:
                    raise ValueError(f"Unknown joint type '{joint_type}' for joint '{first_joint_name}'.")
                return joint_position
        raise ValueError(f"Joint name '{first_joint_name}' not found.")

def my_simulation(franka_file, run_time):
    gs.init(backend=gs.cpu)
    view=gs.options.ViewerOptions(camera_pos=(1,2,3),camera_lookat=(0,0,0), camera_fov=40)
    sim=gs.options.SimOptions(dt=run_time)
    scene = gs.Scene(viewer_options=view,sim_options=sim)

    ground_plane=scene.add_entity(gs.morphs.Plane())
    franka=scene.add_entity(gs.morphs.MJCF(file=franka_file))
    scene.build()

    robot = Robot(name="franka", genesis_entity=franka)    #### also make auto-detection of entity name from xml

    for i in range(300):
        scene.step()

        robot_pose = robot.get_pose()
        robot_velocity = robot.get_velocity()
        first_link_pose = robot.get_single_link_pos()
        all_link_poses = robot.get_all_link_poses()
        first_joint_position = robot.get_first_joint_position()
        all_joint_positions = robot.get_all_joint_positions()

if __name__ == '__main__':
    file,time=get_inputs()
    my_simulation(file,time)
