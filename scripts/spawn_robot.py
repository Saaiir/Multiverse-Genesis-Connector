import argparse
import genesis as gs

def get_inputs() -> str:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Path to the XML",
                        default="/home/abdulrehmansair/MultiverseProject/Multiverse-Genesis-Connector/resources/mjcf/mujoco_menagerie/anybotics_anymal_c/anymal_c.xml")
    parser.add_argument("--time_step", type=float, help="Time step in seconds", default=0.01)
    parser.add_argument("--disable_contact", help="Disable contact", default=False, action="store_true")
    args = parser.parse_args()
    input_xml = args.input
    time_step = args.time_step
    disable_contact = args.disable_contact
    return input_xml, time_step, disable_contact



def run_simulation(input: str, time_step: float, disable_contact: bool) -> None:
    gs.init(backend=gs.cpu)

    scene = gs.Scene(show_viewer=True)
    plane = scene.add_entity(gs.morphs.Plane())
    franka = scene.add_entity(
        gs.morphs.MJCF(file=input),
    )

    scene.build()

    for i in range(200):
        scene.step()

if __name__ == "__main__":
    xml_path, timee, disable = get_inputs()
    run_simulation(xml_path, timee, disable)
