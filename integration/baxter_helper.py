#!/usr/bin/env python
import rospy
import baxter_interface

class Helper(object):
    def __init__(self):
        self._head = baxter_interface.Head()
        self._llimb = baxter_interface.Limb("left")
        self._rlimb = baxter_interface.Limb("right")
        self._lgrip = baxter_interface.Gripper("left")
        self._rgrip = baxter_interface.Gripper("right")
        self._joint_names = ["e0", "e1", "s0", "s1", "w0", "w1", "w2"]

        self._buttons = {
            "rlower": baxter_interface.DigitalIO("right_lower_button"),
            "rupper": baxter_interface.DigitalIO("right_upper_button"),
            "l_itb":  baxter_interface.DigitalIO("left_itb_button0"),
            "r_itb":  baxter_interface.DigitalIO("right_itb_button0"),
            }
        self._rgrip.calibrate()

    def check(self):
        for b in self._buttons:
            if self._buttons[b].state:
                if b == "rlower":
                    self._rgrip.close()
                elif b == "rupper":
                    self._rgrip.open()
                elif b == "l_itb":
                    self.print_joints("left")
                elif b == "r_itb":
                    self.print_joints("right")

                while self._buttons[b].state:
                    rospy.sleep(0.1)

    def print_joints(self, side):
        if side == "left":
            joints = self._llimb.joint_angles()
        elif side == "right":
            joints = self._rlimb.joint_angles()
        else:
            raise RuntimeError("Unknown limb string: %s" % side)
        angles = [joints[side + "_" + n] for n in self._joint_names]
        print(str(angles)[1:-1])

def main():
    rospy.init_node("baxter_helper")
    h = Helper()
    while not rospy.is_shutdown():
        h.check()

if __name__ == "__main__":
    main()
