import unittest
import genesis_connector

class TestGenesisConnector(unittest.TestCase):

    def test_print_genesis(self):
        genesis = genesis_connector.ask_genesis()
        self.assertEqual(genesis, "Genesis")
        self.assertTrue(genesis.startswith("G"))

        result = genesis_connector.plus(2.0, 3.0)
        self.assertEqual(result, 5.0)

        self.assertRaises(TypeError, genesis_connector.plus, 2, "3")

    def test_start_simulation(self):
        simulation = genesis_connector.start_simulation(input="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/franka_emika_panda/panda.xml",
                                           time_step=0.005,
                                           disable_contact=False)

        for step in range(100):
            # simulation.step()
            all_link_names = simulation.get_all_body_names()
            # all_links = simulation.get_all_links()
            # for link in all_links:
            #     link_pos = link.get_pos()
            # all_joints = simulation.get_all_joints()
            # for joint in all_joints:
            #     joint_value = joint.get_value()
            #     joint_value += 0.01
            #     joint.set_value(joint_value)