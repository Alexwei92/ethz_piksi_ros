import os
import datetime
import csv
import rospy
import numpy as np
from collections import deque

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped 
from piksi_rtk_msgs.msg import Heartbeat, ReceiverState_V2_4_1
from piksi_rtk_msgs.srv import RecordGPS


class PiksiMultiRecord():
    TIME_OUT = 3.0 # heartbeat time out

    def __init__(self):
        self.init_variables()
        self.define_subscriber()
        self.define_service()

        # configure output
        output_folder_name = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.output_folder = os.path.join('../data', output_folder_name)
        if not os.path.isdir(self.output_folder):
            os.makedirs(self.output_folder)
    
    def init_variables(self):
        self.heartbeat_time = None
        self.last_fix_mode = None
        self.gps_is_fixed = False

        self.write_into_buffer = False
        self.data_seq = 0
        self.navsatfix_best_fix_buffer = deque(maxlen=100)
        self.enu_pose_best_fix_buffer = deque(maxlen=100)
        self.receiver_state_buffer = deque(maxlen=100)

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

    def define_service(self):
        self.record_data_srv = rospy.Service(
            'record_gps_data',
            RecordGPS,
            self.record_gps_data,
        )

    def record_gps_data(self, req):
        success = False
        try:
            period = req.period
            if period <= 0:
                period = 1.0


            self.clean_buffer() # in case something left in the buffer
            self.write_into_buffer = True
            rospy.sleep(rospy.Duration(period))
            self.write_into_buffer = False

            # calculate average
            lat_avg, lon_avg = self.calculate_average()
            # save to file
            data = {}
            data['seq'] = self.data_seq
            data['lat_avg'] = lat_avg
            data['lon_avg'] = lon_avg
            data['period'] = period
            self.save_data_to_csv(data, filename='gps_at_trigger.csv')
            self.save_buffer_to_csv(self.navsatfix_best_fix_buffer, filename='gps_buffer_at_trigger.csv')
            self.save_buffer_to_csv(self.receiver_state_buffer, filename='gps_state_at_trigger.csv')

            print('*' * 15 + ' Data Seq: %d ' % self.data_seq + '*' * 15)
            print("Record period = %.1f second" % period)
            print("latitude_avg = %f, longitude_avg = %f" % (lat_avg, lon_avg))
            print("Record successfully!")

            self.data_seq += 1
            success = True


        finally:
            self.clean_buffer()

        return success

    def calculate_average(self):
        if len(self.navsatfix_best_fix_buffer) == 0:
            lat_avg, lon_avg = 0, 0

        else:
            lat, lon = [], []
            for i in range(len(self.navsatfix_best_fix_buffer)):
                lat.append(self.navsatfix_best_fix_buffer[i]['latitude'])
                lon.append(self.navsatfix_best_fix_buffer[i]['longitude'])
            
            lat = np.array(lat)
            lon = np.array(lon)

            lat_avg = lat.mean()
            lon_avg = lon.mean()

        return lat_avg, lon_avg

    def save_data_to_csv(self, data, filename='data_at_trigger.csv'):
        csv_file = os.path.join(self.output_folder, filename)
        with open(csv_file, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=data.keys())
            if data['seq'] == 0:
                writer.writeheader()
            writer.writerow(data)

    def save_buffer_to_csv(self, buffer, filename='buffer_at_trigger.csv'):
        if len(buffer) == 0:
            return
        
        csv_file = os.path.join(self.output_folder, filename)
        csv_columns = list(buffer[-1].keys())

        with open(csv_file, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=csv_columns)
            if self.data_seq == 0:
                writer.writeheader()
            for i in range(len(buffer)):
                writer.writerow(buffer[i])

    def clean_buffer(self):
        self.navsatfix_best_fix_buffer.clear()
        self.enu_pose_best_fix_buffer.clear()
        self.receiver_state_buffer.clear()

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
        if self.write_into_buffer:
            self.navsatfix_best_fix_buffer.append(data)

    def enu_pose_best_fix_callback(self, msg):
        data = {}
        data['seq'] = self.data_seq
        data['time'] = msg.header.stamp.to_sec()
        data['x'] = msg.pose.pose.position.x
        data['y'] = msg.pose.pose.position.y
        data['z'] = msg.pose.pose.position.z
        if self.write_into_buffer:
            self.enu_pose_best_fix_buffer.append(data)
    
    def receiver_state_callback(self, msg):
        data = {}
        data['seq'] = self.data_seq
        data['time'] = msg.header.stamp.to_sec()
        data['num_sat'] = msg.num_sat # Number of satellites.
        data['rtk_mode_fix'] = msg.rtk_mode_fix # 1 = Fixed, 0 = Float.
        data['fix_mode'] = msg.fix_mode # Invalid, Single Point Position (SPP), Differential GNSS (DGNSS), Float RTK, Fixed RTK.
        if self.write_into_buffer:
            self.receiver_state_buffer.append(data)

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
                
            else:
                rospy.loginfo_throttle(1, 'Waiting for Piksi Multi...')

            loop_rate.sleep()


if __name__ == '__main__':

    rospy.init_node('record_data', anonymous=True)

    piksi_handler = PiksiMultiRecord()
    piksi_handler.run()  