import pickle
import numpy as np
import os


moveit_joint_names = ["wam/base_yaw_joint",
                      "wam/shoulder_pitch_joint",
                      "wam/shoulder_yaw_joint",
                      "wam/elbow_pitch_joint",
                      "wam/wrist_yaw_joint",
                      "wam/wrist_pitch_joint",
                      "wam/palm_yaw_joint"]

joint_state_joint_names = ["wam/base_yaw_joint",
                           "wam/elbow_pitch_joint",
                           "wam/palm_yaw_joint",
                           "wam/shoulder_pitch_joint",
                           "wam/shoulder_yaw_joint",
                           "wam/wrist_pitch_joint",
                           "wam/wrist_yaw_joint"]


def translate_from_joint_state_order_to_moveit_order(joint_state):
    moveit_goal = joint_state.copy()
    for i, joint_name in enumerate(moveit_joint_names):
        moveit_goal[i] = float(joint_state[joint_state_joint_names.index(joint_name)])
    return moveit_goal


def load_and_preprocess(save_path):
    """
    Load and preprocess demonstration file.
    :param save_path: the path to the saved demonstration.
    :return: a dictionary with two keys "observations" and "actions".
        "observations" is a np array with shape [len_demo, 19]
            The first dimension represents the time sequence of the demonstration.
            The 0-6 indexes of the second dimension are robot joint positions.
            The 7-13 indexes of the second dimension are robot joint velocities.
            The 14-16 indexes of the second dimension are 3D pingpong ball locations in the world.
            The 17-18 indexes of the second dimension are always 1.0, and you do not need to worry about it.
        "action" is a np array with shape [len_demo, 7]
            The first dimension represents the time sequence of the demonstration.
            The second dimension represents the robot joint position differences from the current state, i.e., the commanded next position = current_joint_position + action.
    """
    with open(save_path, "rb") as f:
        data = pickle.load(f)
        processed_data = np.zeros(shape=[len(data["Q"]) - 1, 19])
        # remove the 7th joint in observation, and remove the last timestep
        processed_data[:, :7] = np.array(data["Q"])[:-1, :7]
        processed_data[:, 7:14] = np.array(data["Qdot"])[:-1, :7]
        processed_data[:, 14:17] = np.array(data["pos"])[:-1, :]
        processed_data[:, 17] = 1.0
        processed_data[:, 18] = 1.0
        actions = processed_data[1:, :7] - processed_data[:-1, :7]
        expert = {"observations": processed_data, "actions": actions}
        return expert


def save_video(ims, filename, fps=10.0):
    import cv2
    folder = os.path.dirname(filename)
    if not os.path.exists(folder):
        os.makedirs(folder)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    (height, width, _) = ims[0].shape
    writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    for im in ims:
        writer.write(im)
    writer.release()
