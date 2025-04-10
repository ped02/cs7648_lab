import time
from threading import Thread
import cv2
import os
import pickle  # nosec

import numpy as np  # noqa: F401
import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from wam_environment.utils import save_video
from wam_environment.utils import (
    translate_from_joint_state_order_to_moveit_order,
)


class CallbackManager:
    def __init__(
        self,
        tracking_ball=True,
        tracking_robot=True,
        tracking_debug_image=False,
    ):
        self.ball_position = []
        self.wam_position = []
        self.wam_speed = []
        self.tracking_ball = tracking_ball
        self.tracking_robot = tracking_robot
        self.tracking_debug_image = tracking_debug_image
        self.record_on = False
        self.record_data = {'Q': [], 'Qdot': [], 'pos': [], 'time': []}
        self.record_image_data_1 = []
        self.record_image_data_2 = []
        self.bridge = CvBridge()  # Instantiate CvBridge
        self.time = []
        if tracking_ball:
            rospy.Subscriber('/ball_odometry', Odometry, self._ball_callback)
        if tracking_robot:
            rospy.Subscriber('/joint_states', JointState, self._wam_callback)
        if tracking_debug_image:
            rospy.Subscriber(
                '/ball_detection_1/debug_img', Image, self._image_callback_1
            )
            rospy.Subscriber(
                '/ball_detection_2/debug_img', Image, self._image_callback_2
            )

    def reset(self):
        self.ball_position = []
        self.wam_position = []
        self.wam_speed = []
        self.record_data = {'Q': [], 'Qdot': [], 'pos': [], 'time': []}
        self.record_image_data_1 = []
        self.record_image_data_2 = []
        self.time = []

    def start_recording(self, record_rate=200):
        self.record_data = {'Q': [], 'Qdot': [], 'pos': [], 'time': []}
        self.record_image_data_1 = []
        self.record_image_data_2 = []
        self.record_on = True
        print('Start recording! ')
        Thread(target=self._record, kwargs={'record_rate': record_rate}).start()

    def stop_recording(self):
        if self.record_on:
            self.record_on = False
            print('Recording Stopped. ')
        else:
            print(
                '[Warning] Requested stop_recording but recording is not on. '
            )

    def _ball_callback(self, data: Odometry):
        self.ball_position = [
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
        ]

    def _wam_callback(self, data: JointState):
        self.wam_position = translate_from_joint_state_order_to_moveit_order(
            list(data.position)
        )
        self.wam_speed = translate_from_joint_state_order_to_moveit_order(
            list(data.velocity)
        )
        self.time = time.time()

    def _image_callback_1(self, msg: Image):
        if self.record_on:
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                print('image error!', e)
            else:
                self.record_image_data_1.append(cv2_img)
                rospy.sleep(0.02)

    def _image_callback_2(self, msg: Image):
        if self.record_on:
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                print('image error!', e)
            else:
                self.record_image_data_2.append(cv2_img)
                rospy.sleep(0.02)

    def _record(self, record_rate):
        rate = rospy.Rate(record_rate)
        while self.record_on:
            current_data = self.wam_position.copy()
            current_data.extend(self.wam_speed.copy())
            current_data.extend(self.ball_position.copy())
            current_data.append(self.time)
            if len(current_data) != 18:
                print('[Warning] Not getting size 18 for recording data')
            self.record_data['Q'].append(list(current_data[:7]))
            self.record_data['Qdot'].append(list(current_data[7:14]))
            self.record_data['pos'].append(list(current_data[14:17]))
            self.record_data['time'].append(current_data[17])
            rate.sleep()

    def save_demo(self, saving_name):
        print('Saving demo data...')
        demo_data_path = os.path.join('demos', saving_name + '.pkl')
        with open(demo_data_path, 'wb') as f:
            pickle.dump(self.record_data, f)
        print('Demo data saved at ' + demo_data_path)

        if self.tracking_debug_image:
            if len(self.record_image_data_1) > 0:
                print('Saving demo images/videos...')
                temp_image_path = 'temp/debug_images_1/'
                for i in range(len(self.record_image_data_1)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_1[i],
                    )
                save_video(
                    self.record_image_data_1, temp_image_path + 'video.mp4'
                )
                print('Demo images/videos saved at ' + temp_image_path)

                demo_image_path = 'demos/images/' + saving_name + '/1/'
                os.makedirs(demo_image_path, exist_ok=True)
                for i in range(len(self.record_image_data_1)):
                    cv2.imwrite(
                        demo_image_path + '%d.jpeg' % i,
                        self.record_image_data_1[i],
                    )
                save_video(
                    self.record_image_data_1, demo_image_path + 'video.mp4'
                )
                print('Demo images/videos saved at ' + demo_image_path)

            if len(self.record_image_data_2) > 0:
                temp_image_path = 'temp/debug_images_2/'
                for i in range(len(self.record_image_data_2)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_2[i],
                    )
                save_video(
                    self.record_image_data_2, temp_image_path + 'video.mp4'
                )
                print('Demo images/videos saved at ' + temp_image_path)

                demo_image_path = 'demos/images/' + saving_name + '/2/'
                os.makedirs(demo_image_path, exist_ok=True)
                for i in range(len(self.record_image_data_2)):
                    cv2.imwrite(
                        demo_image_path + '%d.jpeg' % i,
                        self.record_image_data_2[i],
                    )
                save_video(
                    self.record_image_data_2, demo_image_path + 'video.mp4'
                )
                print('Demo images/videos saved at ' + demo_image_path)

    def save_replay_data(self, saving_name):
        print('Saving replay data...')
        replay_data_path = os.path.join('temp', saving_name + '.pkl')
        with open(replay_data_path, 'wb') as f:
            pickle.dump(self.record_data, f)
        print('Replay data saved at ' + replay_data_path)

        if self.tracking_debug_image:
            if len(self.record_image_data_1) > 0:
                print('Saving replay images/videos...')
                temp_image_path = 'temp/debug_images_1/'
                for i in range(len(self.record_image_data_1)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_1[i],
                    )
                save_video(
                    self.record_image_data_1, temp_image_path + 'video.mp4'
                )
                print('Replay images/videos saved at ' + temp_image_path)

            if len(self.record_image_data_2) > 0:
                print('Saving replay images/videos...')
                temp_image_path = 'temp/debug_images_2/'
                for i in range(len(self.record_image_data_2)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_2[i],
                    )
                save_video(
                    self.record_image_data_2, temp_image_path + 'video.mp4'
                )
                print('Replay images/videos saved at ' + temp_image_path)

    def save_record_data_for_ball_calculation(self, saving_name):
        print('Saving replay data...')
        log_dir = (
            'analysis/record_data_for_ball_calculation/' + saving_name + '/'
        )
        os.makedirs(log_dir, exist_ok=True)
        replay_data_path = os.path.join(log_dir, 'data.pkl')
        with open(replay_data_path, 'wb') as f:
            pickle.dump(self.record_data, f)
        print('Replay data saved at ' + replay_data_path)

        if self.tracking_debug_image:
            if len(self.record_image_data_1) > 0:
                print('Saving replay images/videos...')
                temp_image_path = log_dir + 'debug_images_1/'
                os.makedirs(temp_image_path, exist_ok=True)
                for i in range(len(self.record_image_data_1)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_1[i],
                    )
                save_video(
                    self.record_image_data_1, temp_image_path + 'video.mp4'
                )
                print('Replay images/videos saved at ' + temp_image_path)

            if len(self.record_image_data_2) > 0:
                print('Saving replay images/videos...')
                temp_image_path = log_dir + 'debug_images_2/'
                os.makedirs(temp_image_path, exist_ok=True)
                for i in range(len(self.record_image_data_2)):
                    cv2.imwrite(
                        temp_image_path + '%d.jpeg' % i,
                        self.record_image_data_2[i],
                    )
                save_video(
                    self.record_image_data_2, temp_image_path + 'video.mp4'
                )
                print('Replay images/videos saved at ' + temp_image_path)
