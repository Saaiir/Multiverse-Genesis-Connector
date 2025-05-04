import unittest
import genesis_connector
import os

class TestGenesisConnector(unittest.TestCase):
    simulation = None

    @classmethod
    def setUpClass(cls):
        # This method is called once for the entire class
        # You can set up any class-level resources here
        simulation = genesis_connector.start_simulation(input="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/franka_emika_panda/panda.xml",
                                                             time_step=0.005,
                                                             disable_contact=False)
        object_name = "bread_1"
        object_path = "./../resources/objects/bread_roll_1/bread_roll_1.xml"
        position = [0.0, 0.0, 0.5]
        orientation = [0.0, 0.0, 0.0, 1.0]
        simulation.spawn_object(object_name, object_path, position, orientation)
        cls.simulation = simulation

    def test_print_genesis(self):
        genesis = genesis_connector.ask_genesis()
        self.assertEqual(genesis, "Genesis")
        self.assertTrue(genesis.startswith("G"))

        result = genesis_connector.plus(2.0, 3.0)
        self.assertEqual(result, 5.0)

        self.assertRaises(TypeError, genesis_connector.plus, 2, "3")

    def test_start_simulation(self):
        self.assertIsNotNone(self.simulation)
        self.assertIsInstance(self.simulation, genesis_connector.Simulation)


    def test_get_all_body_names(self):
        body_names = self.simulation.get_all_body_names()
        self.assertIsInstance(body_names, list)
        self.assertTrue(len(body_names) > 0)
        self.assertIn("panda", body_names) # Example body name, adjust as needed
    
    def test_get_all_joint_names(self):
        joint_names = self.simulation.get_all_joint_names()
        self.assertIsInstance(joint_names, list)
        self.assertTrue(len(joint_names) > 0)
        self.assertIn("panda_joint", joint_names) # Example joint name, adjust as needed

    def test_set_and_get_joint_position(self):
        joint_name = "panda_joint" # Example joint name, adjust as needed
        position = 0.5
        self.simulation.set_joint_position(joint_name, position)
        # Assuming you have a way to get the current position of the joint
        current_position = self.simulation.get_joint_position(joint_name)
        self.assertEqual(current_position, position)

    def test_set_and_get_joint_velocity(self):
        joint_name = "panda_joint" # Example joint name, adjust as needed
        velocity = 0.1
        self.simulation.set_joint_velocity(joint_name, velocity)
        # Assuming you have a way to get the current velocity of the joint
        current_velocity = self.simulation.get_joint_velocity(joint_name)
        self.assertEqual(current_velocity, velocity)
    
    def test_set_and_get_joint_torque(self):
        joint_name = "panda_joint" # Example joint name, adjust as needed
        torque = 0.5
        self.simulation.set_joint_torque(joint_name, torque)
        # Assuming you have a way to get the current torque of the joint
        current_torque = self.simulation.get_joint_torque(joint_name)
        self.assertEqual(current_torque, torque)

    def test_set_and_get_object_position(self):
        object_name = "bread_1" # Example object name, adjust as needed
        position = [0.0, 0.0, 0.5]
        self.simulation.set_object_position(object_name, position)
        # Assuming you have a way to get the current position of the object
        current_position = self.simulation.get_object_position(object_name)
        self.assertEqual(current_position, position)

    def test_set_and_get_object_velocity(self):
        object_name = "bread_1" # Example object name, adjust as needed
        velocity = [0.0, 0.0, 0.1]
        self.simulation.set_object_velocity(object_name, velocity)
        # Assuming you have a way to get the current velocity of the object
        current_velocity = self.simulation.get_object_velocity(object_name)
        self.assertEqual(current_velocity, velocity)
