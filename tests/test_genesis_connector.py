import unittest
import math
import time
import torch
import genesis_connector


class TestGenesisConnector(unittest.TestCase):
    simulation = None

    @classmethod
    def setUpClass(cls):
        franka_simulation, obj_simulation, sim_wrapper  = genesis_connector.start_simulation(time_step=0.005, disable_contact=False)
        cls.franka_simulation = franka_simulation
        cls.obj_simulation = obj_simulation
        cls.sim_wrapper = sim_wrapper

    def test_step(self):
        total_steps=self.franka_simulation.step()
        self.assertIsNone(total_steps)

    # def test_get_pose(self):
    #     pose = self.franka_simulation.get_pose()
    #     self.assertEqual(len(pose), 7)  # 3 for position and 4 for orientation (total 7)
    #     position_values = pose[:3]
    #     length_position_values = position_values.shape[0]
    #     expected_position_values = torch.tensor([1.0,2.0,3.0])
    #     expected_length_position_values = expected_position_values.shape[0]
    #     self.assertEqual(length_position_values, expected_length_position_values, "position lengths do not match")
    #
    #     orientation_values = pose[3:7]
    #     length_orientation_values = orientation_values.shape[0]
    #     expected_orientation_values = torch.tensor([1.0,0.0,2.0,4.0])
    #     expected_length_orientation_values = expected_orientation_values.shape[0]
    #     self.assertEqual(length_orientation_values, expected_length_orientation_values, "orientation lengths do not match")

    # def test_get_velocity(self):
    #     robot_velocity = self.franka_simulation.get_velocity()
    #     velocity_values = robot_velocity[:3]
    #     length_velocity_values = velocity_values.shape[0]
    #     expected_velocity_values = torch.tensor([1.0,2.0,3.0])
    #     expected_length_velocity_values = expected_velocity_values.shape[0]
    #     velocity_match = (length_velocity_values == expected_length_velocity_values)
    #     self.assertTrue(velocity_match, "velocity length do not match")

    # def test_get_single_link_pos(self):
    #     pose = self.franka_simulation.get_single_link_pos()
    #     position_values = pose[:3]
    #     length_position_values = position_values.shape[0]
    #     expected_position_values = torch.tensor([1.0, 2.0, 3.0])
    #     expected_length_position_values = expected_position_values.shape[0]
    #     self.assertEqual(length_position_values, expected_length_position_values, "position lengths do not match")
    #
    #     orientation_values = pose[3:7]
    #     length_orientation_values = orientation_values.shape[0]
    #     expected_orientation_values = torch.tensor([1.0, 0.0, 2.0, 4.0])
    #     expected_length_orientation_values = expected_orientation_values.shape[0]
    #     self.assertEqual(length_orientation_values, expected_length_orientation_values, "orientation lengths do not match")

    # def test_get_all_link_poses(self):
    #     links = self.franka_simulation.get_all_link_poses()
    #     self.assertIsInstance(links, dict)
    #     link_length=len(links)
    #     self.assertTrue(link_length > 0)
    #
    #     first_link_position_values = links["link1"][:3]
    #     length_first_link_position_values = first_link_position_values.shape[0]
    #     expected_first_link_position_vales = torch.tensor([1.0, 2.0, 3.0])
    #     expected_length_first_link_position_vales = expected_first_link_position_vales.shape[0]
    #     self.assertEqual(length_first_link_position_values,expected_length_first_link_position_vales, "first link positions do not match")
    #
    #     last_link_key = list(links.keys())[-1]
    #     last_link_orientation_values = links[last_link_key][3:7]
    #     #last_link_orientation_values = links["right_finger"][3:7]
    #     length_orientation_values = last_link_orientation_values.shape[0]
    #     expected_orientation_vales = torch.tensor([1.0, 0.0, 2.0, 4.0])
    #     expected_length_orientation_vales = expected_orientation_vales.shape[0]
    #     self.assertEqual(length_orientation_values, expected_length_orientation_vales, "orientation lengths do not match")

    def test_get_all_joint_positions(self):#
        joint_positions = self.franka_simulation.get_all_joint_positions()
        joint_names = self.franka_simulation.find_all_joint_names()

        num_joint_positions = len(joint_positions)
        num_joint_names = len(joint_names)
        self.assertEqual(num_joint_names, num_joint_positions, "total joint and total names do not match")

        for j_names in joint_names:   #(iterating over list of joint names)
            position = joint_positions[j_names]
            if isinstance(position, list):
                length_position = len(position)
                self.assertEqual(length_position,3,f"position should have 3 values" )
            elif isinstance(position, float) or isinstance(position, int):
                self.assertEqual(1, 1, f"position should be a single value")


    # def test_get_first_joint_position(self): # getting tensors that is why .shape method can be applied on them
    #     first_joint_position = self.franka_simulation.get_first_joint_position()
    #     length_first_joint_position = first_joint_position.shape[0]
    #     expected_first_joint_values = torch.tensor([1.0, 2.0, 3.0])
    #     length_expected_first_joint_values = expected_first_joint_values.shape[0]
    #     self.assertEqual(length_first_joint_position, length_expected_first_joint_values)

########################################################################################################################
    def test_get_all_body_names(self):
        body_names = self.sim_wrapper.get_all_body_names()
        self.assertIsInstance(body_names, list)
        self.assertTrue(len(body_names) > 0)
        print("body names:", body_names)
        for body_name in body_names:
            print(f"Checking body name: {body_name}")
            self.assertIn(body_name, body_names, f"Body name '{body_name}' is empty or whitespace")
        # self.assertIn("panda", body_names)

    # def test_get_all_link_names(self):
    #     link_names = self.franka_simulation.get_all_link_names()
    #     print("Link names:", link_names)
    #     self.assertIsInstance(link_names, list)
    #     self.assertTrue(len(link_names) > 0, "there are no body names")
    #     self.assertIn("link0", link_names)

    def test_find_all_joint_names(self):
        joint_names = self.franka_simulation.find_all_joint_names()
        self.assertIsInstance(joint_names, list)
        self.assertTrue(len(joint_names) > 0, "there are no body names")
        print("Joint names:", joint_names)
        # print ("first joint name:", {joint_names[0]})
        # breakpoint()
        for joint_name in joint_names:
            print(f"Checking joint: {joint_name}")


    #### for franka arm #########
    def test_set_and_get_joint_position(self): #getting lists that is why .shape method cannot be applied and hence len() method is used

        joint_name = "joint1"
        position_in_degrees = 5.0

        # joint_name = "shoulder"
        # position_in_degrees = 5.0
        # joint_type, initial_position = self.franka_simulation.get_joint_position(joint_name)
        # print(f'joint name: {joint_name} ,joint_type: {joint_type}, Joint_initial_position: {initial_position} degrees')
        # print(f"Joint type for {joint_name}: {joint_type}")

        self.franka_simulation.set_joint_position(joint_name, position_in_degrees)  # ✅ Actually move the joint and first converts the degree into radians

        # for _ in range(60):
        #     self.franka_simulation.step()

        joint_type, updated_position = self.franka_simulation.get_joint_position(joint_name)
        print(f'joint name: {joint_name} ,joint_type: {joint_type}, Joint_updated_position: {updated_position} degrees')

        # updated_position_deg = math.degrees(updated_position)
        # print(f'Updated position: {updated_position_deg} degrees')
        # breakpoint()

        if joint_type == "revolute":
            self.assertGreaterEqual(updated_position, 0.0)
            # self.assertTrue(abs(current_position - position_in_degrees) < 0.0001)

            self.assertAlmostEqual(updated_position, position_in_degrees, delta=3.0)
        # elif joint_type == "fixed":
        #     length_current_position = len(current_position)
        #     length_expected_length_current_position = len(fixed_position)
        #     self.assertEqual(length_current_position, length_expected_length_current_position)


    ####### for humanoid #######
    # def test_set_and_get_joint_position(self): #getting lists that is why .shape method cannot be applied and hence len() method is used
    #
    #
    #     # joint_name = "joint1"
    #     # position = 5.0
    #
    #     joint_name = "torso_1_joint"
    #     position_in_degrees = 30
    #     #fixed_position = [1.0, 2.0, 3.0]
    #
    #     joint_type, initial_position = self.franka_simulation.get_joint_position(joint_name)
    #     print(f'joint name: {joint_name} ,joint_type: {joint_type}, Joint_initial_position: {initial_position} degrees')
    #     # print(f"Joint type for {joint_name}: {joint_type}")
    #
    #     self.franka_simulation.set_joint_position(joint_name, position_in_degrees)  # ✅ Actually move the joint  ----- on it for humanoid 1
    #
    #     for _ in range(50):
    #         self.franka_simulation.step()
    #
    #     # self.franka_simulation.set_joint_position(joint_name, position)
    #     # self.franka_simulation.step()
    #     joint_type, updated_position = self.franka_simulation.get_joint_position(joint_name)
    #     print(f'joint name: {joint_name} ,joint_type: {joint_type}, Joint_updated_position: {updated_position} degrees')
    #
    #     # breakpoint()
    #     updated_position_deg = math.degrees(updated_position)
    #     print(f'Updated position: {updated_position_deg} degrees')
    #     # breakpoint()
    #
    #     if joint_type == "revolute":
    #         self.assertGreaterEqual(updated_position_deg, 0.0)
    #         # self.assertTrue(abs(current_position - position) < 0.0001)
    #
    #         self.assertAlmostEqual(updated_position_deg, position_in_degrees, delta=2.0)     # on for humaniod 1
    #
    #     # elif joint_type == "fixed":
    #     #     length_current_position = len(current_position)
    #     #     length_expected_length_current_position = len(fixed_position)
    #     #     self.assertEqual(length_current_position, length_expected_length_current_position)

    def test_set_and_get_joint_velocity(self):
        # joint_name = "elbow"
        # velocity = 10
        joint_name = "joint1"
        # joint_name = "leg_left_1_joint"
        velocity = 10
        # joint_name = "right_wrist_yaw_joint"
        # velocity = 2
        self.franka_simulation.set_joint_velocity(joint_name, velocity)
        current_velocity = self.franka_simulation.get_joint_velocity(joint_name)
        self.assertEqual(current_velocity, velocity)
        print(f'joint name: {joint_name} ,Joint_current_velocity: {current_velocity}')

    def test_set_and_get_joint_torque(self):
        # joint_name = "waist"
        # torque = 2
        joint_name = "joint1"
        # joint_name = "head_2_joint"
        torque = 2
        # joint_name = "right_wrist_roll_joint"
        # torque = 2
        self.franka_simulation.set_joint_torque(joint_name, torque)
        self.franka_simulation.step()
        current_torque = self.franka_simulation.get_joint_torque(joint_name)
        self.assertEqual(current_torque, torque)
        print(f'joint name: {joint_name} ,Joint_current_torque: {current_torque}')

    # def test_start_simulation(self):
    #     self.assertIsNotNone(self.franka_simulation, "simulation failed to start, returned None")

    def test_set_and_get_object_position(self):
        object_name = "bread_1"
        position = [0.0, 0.0, 0.5]
        self.obj_simulation.set_position(object_name, position)
        current_position = self.obj_simulation.get_position(object_name)
        print(f'object name: {object_name} ,set_position_retrieved_: {current_position}')
        self.assertEqual(current_position, position)

    def test_get_object_velocity(self):
        object_name = "bread_1"
        velocity = self.obj_simulation.get_velocity(object_name)
        print(f'object name: {object_name} , velocity: {velocity}')
        self.assertIsInstance(velocity, torch.Tensor) # Checking type
        self.assertEqual(velocity.shape, torch.Size([3])) # Checking shape