import rospy
import serial
from control_msgs.msg import GripperCommand

class GripperControl(object):
    """
    This node is responsible for all communication with the arduino controlling
    the gripper.

    Subscibed topics:
        'cmd' : Listens for control_msgs/GripperCommand messages

    Published topics:
        'curr_cmd' :The last commanded GripperCommand message

    Private parameters:
        'hz' : Rate of command to the arduino
        'device' : The dev file used for serial communication
        'baud_rate'
    """
    def __init__(self):
        self._device   = rospy.get_param('~device', '/dev/ttyUSB0')
        self._baudrate = rospy.get_param('~baud_rate', 115200)
        self._maxangle = rospy.get_param('~max_angle', 165)
        self._minangle = rospy.get_param('~min_angle', 20)

        self._cmd_sub = rospy.Subscriber(
            "cmd", 
            GripperCommand, 
            self.cmd_callback, 
            queue_size=10
        )
        self._curr_cmd = GripperCommand(0.0, 0.0)
        self._arduino = self.connect()
        if self._arduino == None:
            rospy.signal_shutdown('Fatal error')

    def spin(self):
        rospy.spin()

    def cmd_callback(self, cmd):
        acmd = cmd.position*(self._maxangle - self._minangle) + self._minangle
        to_arduino(int(round(acmd)))
        self._curr_cmd = cmd

    def connect(self):
        try:
            arduino = serial.Serial(self._device, self._baudrate, timeout=1)
            #Check for connectivity
            #arduino.write(b'999')
            #resp = arduino.read(3)
            #if len(resp) < 3 or resp[0] != '9':
            #    rospy.logfatal('Could not establish communication with arduino')
            #    return None
            return arduino
        except serial.SerialException as ex:
            rospy.logfatal('Serial port exception: %s' % ex.message)
            return None

    def to_arduino(self, cmd):
        self._arduino.write(bytes(cmd))
        #resp = self._arduino.read(1)
        #if len(resp) != 1 or ord(resp) != cmd:
        #    rospy.logwarn('No acknowledgement of command: %u' % cmd)

    def test(self):
        startang = 60
        stopang = 120
        angvel = 10 #deg / sec

        rate = rospy.Rate(10.0)
        curr_ang = startang
        while not rospy.is_shutdown():
            if angvel > 0:
                curr_ang = curr_ang + 1
                if curr_ang >= stopang:
                    angvel = -angvel
            else:
                curr_ang = curr_ang - 1
                if curr_ang <= startang:
                    angvel = -angvel
            self.to_arduino(curr_ang)
            rate.sleep()

def main():
    rospy.init_node("gripper_controller")
    grip = GripperControl()
    if len(sys.argv) >= 2 and sys.argv[1] == 'test':
        grip.test()
    else:
        grip.spin()

if __name__ == "__main__":
    main()
