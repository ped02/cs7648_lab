import numpy as np
import rospy
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from wam_environment.utils import translate_from_joint_state_order_to_moveit_order


class StateVerifier:
    def __init__(self):
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name = ["wam/base_yaw_joint",
                                    "wam/shoulder_pitch_joint",
                                    "wam/shoulder_yaw_joint",
                                    "wam/elbow_pitch_joint",
                                    "wam/wrist_yaw_joint",
                                    "wam/wrist_pitch_joint",
                                    "wam/palm_yaw_joint"]
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states_received = False

        # subscribe to joint joint states
        rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')

    def checkCollision(self, position=None):
        """
        check if a position is in collision
            if position is None, then check the current robot state
        """
        if position is None:
            if self.joint_states_received:
                position = self.rs.joint_state.position
            else:
                raise NotImplementedError

        if self.getStateValidity(position).valid:
            rospy.logdebug('position %s not in collision, all ok!' % position)
            return False
        else:
            rospy.logwarn('position %s in collision' % position)
            return True

    def jointStatesCB(self, msg):
        """
        update robot state in self.rs, which will be used
        """
        self.rs.joint_state.position = translate_from_joint_state_order_to_moveit_order(list(msg.position))
        self.joint_states_received = True

    def getStateValidity(self, position, group_name='arm', constraints=None):
        """
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        """
        gsvr = GetStateValidityRequest()

        rs = RobotState()
        rs.joint_state.name = self.rs.joint_state.name
        rs.joint_state.position = position

        gsvr.robot_state = rs
        gsvr.group_name = group_name
        if constraints is not None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result

    def start_collision_checker(self):
        while not self.joint_states_received:
            rospy.sleep(0.1)
        rospy.loginfo('joint states received! continue')
        self.checkCollision()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('collision_checker_node', anonymous=False)
    collision_checker_node = StateVerifier()
    positions_angle = np.array([40.0, 68, -46, 89, -131, -57, -26])  # moveit order
    position_radius = positions_angle / 180 * np.pi
    collision_checker_node.checkCollision(position_radius)
    # collision_checker_node.start_collision_checker()
