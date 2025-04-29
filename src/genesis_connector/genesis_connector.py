def print_genesis():
    print("Genesis")

def ask_genesis():
    return "Genesis"

def plus(a, b):
    if isinstance(a, float) and isinstance(b, float):
        return a + b
    else:
        raise TypeError

import genesis as gs

class Simulation:
    def __init__(self, scene):
        self.scene = scene

    def get_all_body_names(self):
        link_names = []
        for link in self.entity.links:
            link_names.append(link.name)
        return link_names


def start_simulation(input: str, time_step: float, disable_contact: bool) -> Simulation:
    gs.init(backend=gs.cpu)

    scene = gs.Scene(show_viewer=True)
    plane = scene.add_entity(gs.morphs.Plane())
    franka = scene.add_entity(
        gs.morphs.MJCF(file=input),
    )

    scene.build()
    simulation = Simulation(scene)
    return simulation