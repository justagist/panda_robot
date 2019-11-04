
import franka_interface

class FrankaArm(franka_interface.ArmInterface):

    def __init__(self, limb = None, on_state_callback = None):


        self._configure(on_state_callback)


    def _configure(self, on_state_callback):
        franka_interface.ArmInterface.__init__(self)


