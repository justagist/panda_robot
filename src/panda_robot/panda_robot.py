import rospy
import franka_interface

class PandaArm(franka_interface.ArmInterface):

    def __init__(self, limb = None, on_state_callback = None, reset_frames = True):


        self._configure(on_state_callback)

        if reset_frames:
            self.set_EE_frame_to_link('panda_hand' if self.has_gripper else 'panda_link8')


    def _configure(self, on_state_callback):
        franka_interface.ArmInterface.__init__(self)


        self._configure_gripper(self.get_robot_params().get_gripper_joint_names())

        if self.get_robot_params()._in_sim:
            self._frames_interface = None # Frames interface is not implemented for simulation controller
        
    def _configure_gripper(self, gripper_joint_names):

        self._gripper = franka_interface.GripperInterface(ns = self._ns, gripper_joint_names = gripper_joint_names)
        if not self._gripper.exists:
            self._gripper = None
            return

    def get_gripper(self):
        return self._gripper

    @property
    def has_gripper(self):
        return self._gripper is not None