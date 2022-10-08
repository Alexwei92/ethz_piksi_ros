import rospy
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped 
from piksi_rtk_msgs.msg import Heartbeat, ReceiverState_V2_4_1

from utils.plot_utils import GPSMapClass

class PiksiMultiRecord():
    TIME_OUT = 3.0 # heartbeat time out

    def __init__(self):
        self.init_variables()
        self.define_subscriber()

        self.disp_handler = GPSMapClass()
    
    def init_variables(self):
        self.heartbeat_time = None
        self.last_fix_mode = None
        self.gps_is_fixed = False

        self.data_seq = 0
        self.navsatfix_best_fix_data = None
        self.enu_pose_best_fix_data = None
        self.receiver_state_data = None

    def define_subscriber(self):
       # piksi multi heartbeat;
        rospy.Subscriber(
            '/piksi/heartbeat',
            Heartbeat,
            self.heartbeat_callback,
            queue_size=10,
        )

        # this message contains WGS 84 coordinates with best available fix at the moment (either RTK or SPP);
        rospy.Subscriber(
            '/piksi/navsatfix_best_fix',
            NavSatFix,
            self.navsatfix_best_fix_callback,
            queue_size=10,
        )

        # this message contains ENU (East-North-Up) coordinate of the receiver with best available fix at the moment (either RTK or SPP). Orientation is set to identity quaternion (w=1);        rospy.Subscriber(
        rospy.Subscriber(
            '/piksi/enu_pose_best_fix',
            PoseWithCovarianceStamped,
            self.enu_pose_best_fix_callback,
            queue_size=10,
        )

        # this message contains ENU (East-North-Up) coordinate of the receiver with best available fix at the moment (either RTK or SPP). Orientation is set to identity quaternion (w=1);        rospy.Subscriber(
        rospy.Subscriber(
            '/piksi/debug/receiver_state',
            ReceiverState_V2_4_1,
            self.receiver_state_callback,
            queue_size=10,
        )

    # callback
    def heartbeat_callback(self, msg):
        if self.heartbeat_time is None:
            rospy.loginfo('Recevied Piksi Multi heartbeat!')
        self.heartbeat_time = msg.header.stamp


    def navsatfix_best_fix_callback(self, msg):
        data = {}
        data['seq'] = self.data_seq
        data['time'] = msg.header.stamp.to_sec()
        data['latitude'] = msg.latitude
        data['longitude'] = msg.longitude
        data['altitude'] = msg.altitude
        data['status'] = msg.status.status
        data['service'] = msg.status.service
        self.navsatfix_best_fix_data = data

    def enu_pose_best_fix_callback(self, msg):
        data = {}
        data['seq'] = self.data_seq
        data['time'] = msg.header.stamp.to_sec()
        data['x'] = msg.pose.pose.position.x
        data['y'] = msg.pose.pose.position.y
        data['z'] = msg.pose.pose.position.z
        self.enu_pose_best_fix_data = data
    
    def receiver_state_callback(self, msg):
        data = {}
        data['seq'] = self.data_seq
        data['time'] = msg.header.stamp.to_sec()
        data['num_sat'] = msg.num_sat # Number of satellites.
        data['rtk_mode_fix'] = msg.rtk_mode_fix # 1 = Fixed, 0 = Float.
        data['fix_mode'] = msg.fix_mode # Invalid, Single Point Position (SPP), Differential GNSS (DGNSS), Float RTK, Fixed RTK.
        self.receiver_state_data = data

        if data['rtk_mode_fix'] == 0:
            rospy.logwarn_throttle(2.0, 'RTK is float!')
            self.gps_is_fixed = False
        else:
            self.gps_is_fixed = True

        if data['fix_mode'] != self.last_fix_mode:
            rospy.logwarn('Current fix mode: ' + data['fix_mode'])
            self.last_fix_mode = data['fix_mode']

    def run(self, rate=15):
        loop_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if self.heartbeat_time is not None:
                # Heartbeat Watchdog
                if (rospy.Time.now() - self.heartbeat_time).to_sec() > self.TIME_OUT:
                    rospy.logerr('Heartbeat Time Out!')
                    break

                # update graph
                self.disp_handler.update_graph(self.navsatfix_best_fix_data)
                plt.pause(1e-3)
                
            else:
                rospy.loginfo_throttle(1, 'Waiting for Piksi Multi...')

            loop_rate.sleep()


if __name__ == '__main__':

    rospy.init_node('record_data', anonymous=True)

    piksi_handler = PiksiMultiRecord()
    piksi_handler.run()  