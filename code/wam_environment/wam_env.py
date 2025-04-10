#!/usr/bin/env python3
import numpy as np
from time import sleep, time
import os

# from wam_environment.CallbackManager import CallbackManager
# from wam_environment.check_state_validity import StateVerifier
# from std_msgs.msg import Float64MultiArray
# import rospy
# import pickle
# import gym

class WAMEnv():
    def __init__(self, horizon=100, dt=0.02, ball_return_flags=True, position_control=True, velocity_control=False):
        # Serializable.quick_init(self, locals())
        # rospy.init_node("WAM_ENV")
        self.order_of_joint_names = ["wam/base_yaw_joint",
                                     "wam/shoulder_pitch_joint",
                                     "wam/shoulder_yaw_joint",
                                     "wam/elbow_pitch_joint",
                                     "wam/wrist_yaw_joint",
                                     "wam/wrist_pitch_joint",
                                     "wam/palm_yaw_joint"]

        assert position_control or velocity_control
        assert not (position_control and velocity_control)
        self.position_control = position_control
        self.velocity_control = velocity_control

        self.ball_return_flags = ball_return_flags
        # logger.log("Using ball_return_flags=%r" % ball_return_flags)
        # ignoring joint 7 in both observation and action as it's irrelevant.
        # hard joint limit low: [-2.6, -2.0, -2.8, -0.9, -4.76, -1.6, -3.0]
        # hard joint limit high: [2.6, 2.0, 2.8, 3.1, 1.24, 1.6, 3.0]
        # using joint limits got from demos (a bit smaller than the actual hard joint limits)
        # joint position + joint velocity + ball position
        # demo joint low: [1.13405008, -0.29666917, -0.71834508, 1.13429357, -2.41404296, -0.33668506, -0.03226189]
        # demo joint high: [2.11170717, 0.46068294, 1.88204678  2.67781913, -0.93549107, 0.33731763, 0.23281986]
        # demo velocity low: [-0.11016903, -1.29880898, -0.68792177, -3.18350045, -3.18552497, -4.09929268, -0.87090278]
        # demo velocity high: [2.67509339, 1.60940834, 6.64290153, 6.20163291, 3.76027639, 2.73010612, 3.17396976]
        # demo ball low: [-1.81521594, -0.83832843, 0.01538451]
        # demo ball high: [2.3689483, 0.03755151, 0.83166434]
        low = np.array([1.0, -0.5, -1.0, 1.0, -3.0, -0.5, -0.5] + [-1, -1.5, -1, -4, -4, -4.5, -1] + [-2, -1, 0.0])
        high = np.array([2.2, 1.0, 2.0, 3.0, 0.0, 0.5, 0.5] + [3, 2, 7, 7, 4, 3, 4] + [3.0, 1.0, 1.0])
        if ball_return_flags:
            # + [hit_ball, over_net]
            low = np.concatenate([low, [0.0, 0.0]])
            high = np.concatenate([high, [1.0, 1.0]])
        self._observation_space = []#Box(low, high)
        # logger.log("observation space: {}".format(self._observation_space))

        if position_control:
            # action (position delta in one frame) space is calculated by demonstration's limits
            # demo position delta low: [-0.00153398, -0.01876072, -0.00998905, -0.03647465, -0.04222401, -0.05922431, -0.01181566]
            # demo position delta high: [0.03875128, 0.02296899, 0.09893266, 0.09255017, 0.05519168, 0.03779602, 0.02342583]
            self._action_space = [np.array([1.0, -0.5, -1.0, 1.0, -3.0, -0.5, -0.5, -1, -1.5, -1, -4, -4, -4.5, -1]),
                                  np.array([2.2, 1.0, 2.0, 3.0, 0.0, 0.5, 0.5, 3, 2, 7, 7, 4, 3, 4])] 
                                  # Box(np.array([-0.01, -0.16, -0.02, -0.1, -0.07, -0.11, -0.04]),
                                    # np.array([0.11, 0.03, 0.44, 0.28, 0.10, 0.10, 0.04]))
        elif velocity_control:
            raise NotImplementedError

        # logger.log("action space: {}".format(self._action_space))
        self._horizon = horizon

        self.timestep = 0
        self.init_state = [1.46, 0.209, -0.60, 1.173, -0.13, 0.08, 1.55] #None   # Kin Man: hardcoded it 
        self.dt = dt
        self.control_rate = int(1 / self.dt)
        self.seconds_to_move_to_initial_position = 3

        # self.callback_manager = CallbackManager(tracking_ball=True,
        #                                         tracking_robot=True,
        #                                         tracking_debug_image=True)
        # self.wam_commander = rospy.Publisher('/joint_position_controller/command', Float64MultiArray, queue_size=1)
        # self.rospy_control_rate = rospy.Rate(self.control_rate)
        # self.state_verifier = StateVerifier()

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    @property
    def horizon(self):
        return self._horizon

    def move_to_initial_position(self):
        assert self.init_state is not None
        assert len(self.init_state) == 7

        # goal and current position
        joint_goal = self.init_state.copy()
        current_position = self.callback_manager.wam_position.copy()
        while len(current_position) == 0:
            current_position = self.callback_manager.wam_position.copy()
        # calculate linear interpolate trajectory from current position to goal
        trajectory_to_initial_position = []
        differences = [joint_goal[i] - current_position[i] for i in range(len(current_position))]
        num_frames_to_initial_position = self.control_rate * self.seconds_to_move_to_initial_position
        for t in range(self.control_rate):  # fade-in part for a second
            trajectory_to_initial_position.append(
                [current_position[i] + differences[i] * 1.0 / num_frames_to_initial_position / 2.0 * t
                 for i in range(len(current_position))])
        for t in range(num_frames_to_initial_position - int(self.control_rate // 2)):
            trajectory_to_initial_position.append([current_position[i] + differences[i] * 1.0 /
                                                   num_frames_to_initial_position * (t + int(self.control_rate // 2))
                                                   for i in range(len(current_position))])
        for state in trajectory_to_initial_position:
            if self.state_verifier.checkCollision(state):
                print("Detect Collision in trajectory_to_initial_position!!!")
                # rospy.logwarn("Detect Collision in trajectory_to_initial_position!!!")
        input("Press enter to start moving to start position in %d seconds." % self.seconds_to_move_to_initial_position)
        print("[Alert] Moving to start position")
        # for each_data in trajectory_to_initial_position:
            # message = Float64MultiArray(data=each_data)
            # self.wam_commander.publish(message)
            # self.rospy_control_rate.sleep()
        print("Done moving to start position")

    def reset(self):
        self.timestep = 0
        self.callback_manager.stop_recording()
        self.move_to_initial_position()
        self.callback_manager.reset()

        input("Press Enter to start waiting for ball and start execution.")
        print("Waiting for ball...")
        # while len(self.callback_manager.ball_position) == 0:
        #     rospy.sleep(0.01)
        print("Found ball!")

        self.callback_manager.start_recording(record_rate=100)

        return self.get_current_state()

    def get_current_state(self):
        # current_state = np.concatenate([self.callback_manager.ball_position,
        #                                 self.callback_manager.wam_position[:7],
        #                                 self.callback_manager.wam_speed[:7]])
        current_state = np.concatenate([self.callback_manager.wam_position[:7],
                                        self.callback_manager.wam_speed[:7]])

        if self.ball_return_flags:
            current_state = np.concatenate([current_state, [1.0, 1.0]])
        return current_state

    def remember_init_state(self, init_state):
        # !!!!!!!!! init_state has to be in moveit order !!!!!!!!!
        if type(init_state) is np.ndarray:
            init_state = init_state.tolist()
        assert len(init_state) == 7
        self.init_state = init_state

    def step(self, action):
        self.timestep += 1
        done = False
        if self.timestep >= self._horizon:
            done = True

        action_np = action if type(action) is np.ndarray else np.array(action)
        original_action = action_np.copy()
        # if action output by the nn should be [-1, 1] after tanh (we are not doing this option for now)
        # action_np = action_np * self.action_space.high

        # current version of action output by the nn is not bounded
        # we either do assert or clip
        # assert self.action_space.contains(action_np)
        # action_np = np.clip(action_np, self.action_space[0], self.action_space[1])
        if not np.all(action_np == original_action):
            print("action clipped from", original_action, "to", action_np, "!")
        # joint_position_now = self.callback_manager.wam_position
        joint_position_after_move = action_np[:7]
        # clip the command if the target state exceeds set limit (not using now since we have the collision check)
        # joint_position_after_move = np.clip(joint_position_after_move,
        #                                     self.observation_space.low[:7],
        #                                     self.observation_space.high[:7])
        # print("action requested:", action)
        # print("action executed:", joint_position_after_move - joint_position_now[:7])
        joint_position_after_move = joint_position_after_move.tolist()
        print(f"joint_position_after_move: {joint_position_after_move}")
        if self.state_verifier.checkCollision(joint_position_after_move):
            # rospy.logwarn("action %s will result in Collision!!!" % action)
            print("action %s will result in Collision!!!" % action)
            print("proceeding without executing the action")
            return (self.get_current_state(), 0.0, done)

        # execute
        # message = Float64MultiArray(data=joint_position_after_move)
        # self.wam_commander.publish(message)
        self.rospy_control_rate.sleep()

        # assert self._observation_space.contains(robot_state)
        # may_be_killed = False
        # if np.all(np.abs(robot_state[7:]) < 1e-2) and not np.all(np.abs(action) < 1e-2) and self.timestep > 5:
        #     print("WARNING: SEEMS NOT MOVING", "robot_state:", robot_state, "action:", action)
        #     may_be_killed = True
        # print("Step takes time", time() - start_time)

        return (self.get_current_state(), 0.0, done)

    def render(self, *args, **kwargs):
        raise NotImplementedError

    def terminate(self):
        self.callback_manager.stop_recording()
