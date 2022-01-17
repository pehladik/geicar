import math
import rospy
from stm32_ros_msgs.srv import emergency_stop
import radar_ros_msgs.msg as radar
from std_msgs.msg import Float32

TRIGGER_DIST_LONG = "/trigger_dist_long"
TRIGGER_DIST_LAT = "/trigger_dist_lat"


class EmergencyStop:
    def __init__(self):
        if not rospy.has_param(TRIGGER_DIST_LONG):
            rospy.set_param(TRIGGER_DIST_LONG, 5)
        if not rospy.has_param(TRIGGER_DIST_LAT):
            rospy.set_param(TRIGGER_DIST_LAT, 2)

        self.emergency_stop_srv = rospy.ServiceProxy('stm32/emergency_stop', emergency_stop)
        self.prev_us_measure = rospy.get_param(TRIGGER_DIST_LONG) + 1
        self.object_detected_radar = False
        self.object_detected_us = False
        self.emergency_stop_state = False

    def send_emergency_stop(self, stop):
        print(f"setting emergency stop {stop}")
        try:
            success = self.emergency_stop_srv(stop)
            if not success:
                print("Failed to set emergency brakes")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def update_emergency_stop(self):
        new_state = self.object_detected_us or self.object_detected_radar
        if new_state != self.emergency_stop_state:
            self.send_emergency_stop(new_state)
            self.emergency_stop_state = new_state

    def callback_us(self, measure):
        trigger_dist_long = rospy.get_param(TRIGGER_DIST_LONG)
        self.object_detected_us = self.prev_us_measure < trigger_dist_long or measure.data < trigger_dist_long
        self.prev_us_measure = measure.data
        self.update_emergency_stop()

    def callback_radar(self, frame):
        self.object_detected_radar = object_in_trigger_zone(frame)
        self.update_emergency_stop()


def object_in_trigger_zone(frame):
    trigger_dist_long = rospy.get_param(TRIGGER_DIST_LONG)
    trigger_dist_lat = rospy.get_param(TRIGGER_DIST_LAT)
    for obj in frame.objects:
        if obj.distance_long < trigger_dist_long and abs(obj.distance_lat) < (trigger_dist_lat / 2):
            return True
    return False


if __name__ == "__main__":
    rospy.init_node('ros_emergency_stop')

    emergency_stop = EmergencyStop()
    rospy.Subscriber("stm32/ultrasound", Float32, emergency_stop.callback_us)
    rospy.Subscriber("radar_frames", radar.frame, emergency_stop.callback_radar)

    rospy.spin()
