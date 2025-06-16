import unittest
import torch
import genesis_connector


class TestGenesisConnector(unittest.TestCase):
    simulation = None

    @classmethod
    def setUpClass(cls):
        robots, obj_simulation, sim_wrapper  = genesis_connector.start_simulation(time_step=0.005, disable_contact=False)
        cls.robots = robots
        cls.obj_simulation = obj_simulation
        cls.sim_wrapper = sim_wrapper

    def test_get_all_body_names(self):
        expected_body_names = ["panda", "talos"]  # expected body names across all robots
        body_names = self.sim_wrapper.get_all_body_names()
        self.assertIsInstance(body_names, list)
        self.assertTrue(len(body_names) > 0)
        for expected_body in expected_body_names:
            self.assertIn(expected_body, body_names)

    def test_find_all_joint_names(self):
        expected_joint_names = [["joint1"],["torso_1_joint"]]
        for idx, robot in enumerate(self.robots):
            with self.subTest(robot=idx):
                joint_names = robot.find_all_joint_names()
                self.assertIsInstance(joint_names, list)
                self.assertTrue(len(joint_names) > 0)
                for expected_joint in expected_joint_names[idx]:
                    self.assertIn(expected_joint, joint_names)

    def test_set_and_get_joint_position(self):
        joint_name = "torso_1_joint"
        position_in_radians = 0.5236
        for idx, robot in enumerate(self.robots):
            with self.subTest(robot=idx):
                if joint_name in robot.find_all_joint_names():
                    robot.get_joint_position(joint_name)
                    robot.set_joint_position(joint_name, position_in_radians)
                    joint_type, current_position = robot.get_joint_position(joint_name)
                    if joint_type == "revolute":
                        self.assertAlmostEqual(current_position.item(), position_in_radians, places=4)

    def test_set_and_get_joint_velocity(self):
        joint_name = "joint1"
        velocity = 10
        for idx, robot in enumerate(self.robots):
            with self.subTest(robot=idx):
                if joint_name in robot.find_all_joint_names():
                    robot.get_joint_velocity(joint_name)
                    robot.set_joint_velocity(joint_name, velocity)
                    current_velocity = robot.get_joint_velocity(joint_name)
                    self.assertEqual(current_velocity, velocity)

    def test_set_and_get_joint_torque(self):
        joint_name = "leg_right_1_joint"
        torque = 3
        for idx, robot in enumerate(self.robots):
            with self.subTest(robot=idx):
                if joint_name in robot.find_all_joint_names():
                    robot.get_joint_torque(joint_name)
                    robot.set_joint_torque(joint_name, torque)
                    robot.step()
                    current_torque = robot.get_joint_torque(joint_name)
                    self.assertEqual(current_torque, torque)

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