#!/usr/bin/env python

import rospy
from pepper_pose_for_nav.srv import MoveHeadAtPosition, MoveHeadAtPositionResponse
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class HeadFix:
    _continuous_fix_activated = True
    HEAD_PITCH_OBSTACLE = 0.6
    HEAD_YAW_OBSTACLE_LEFT = 0.7
    HEAD_YAW_OBSTACLE_RIGHT = -0.7

    def __init__(self):
        """
        Program to move the head of Pepper in simulation
        """
        self._last_cmd_vel = rospy.Time.now()
        self._last_head_data_update = rospy.Time.now()
        self._error = 0.1
        self._default_pitch = 0.3
        self._default_yaw = 0.0
        self.current_pitch = self._default_pitch
        self.current_yaw = self._default_yaw
        self._rate = rospy.Rate(2)

        self.jt = JointTrajectory()
        self.jt.joint_names = ["HeadPitch", "HeadYaw"]
        self.pt = JointTrajectoryPoint()
        self.jt.points.append(self.pt)

        # declare ros service
        self.setHeadPositionSrv = rospy.Service('move_head_pose_srv', MoveHeadAtPosition, self.setHeadPositionSrvCallback)
        self.head_control = rospy.Publisher('/pepper/Head_controller/command', JointTrajectory, queue_size=10)
        self.cmdSub = rospy.Subscriber("/pepper/cmd_vel", Twist, self.cmdVelCallBack)
        self.head_data_sub = rospy.Subscriber("/pepper/Head_controller/state", JointTrajectoryControllerState, self.update_head_data)

    def fix_head(self):
        while not rospy.is_shutdown():
            if self._continuous_fix_activated and self.needToFixHead(self.current_pitch, self.current_yaw):
                self.publish_head_control(self._default_pitch, self._default_yaw)

            self._rate.sleep()

    def update_head_data(self, data):
        if rospy.Time.now() - self._last_head_data_update > rospy.Duration(2):
            self._last_head_data_update = rospy.Time.now()
            for i in range(len(data.joint_names)):
                if data.joint_names[i] == "HeadPitch":
                    self.current_pitch = data.actual.positions[i]
                elif data.joint_names[i] == "HeadYaw":
                    self.current_yaw = data.actual.positions[i]
                else:
                    pass
            print("{} position : {}".format(data.joint_names, data.actual.positions))

    def needToFixHead(self, headPitchPos, headYawPos):
        result = False
        if headPitchPos > (self._default_pitch + self._error) or headPitchPos < (self._default_pitch - self._error):
            result = True
        if headYawPos > (self._default_yaw + self._error) or headYawPos < (self._default_yaw - self._error):
            result = True
        return result

    def setHeadPositionSrvCallback(self, req):
        self._continuous_fix_activated = req.continuous_fix

        if not self._continuous_fix_activated:
            self.publish_head_control(req.pitch_value, req.yaw_value)

        return MoveHeadAtPositionResponse(True)

    def cmdVelCallBack(self, data):
        self._continuous_fix_activated = False
        self._last_cmd_vel = rospy.Time.now()

        pitch_value = self.HEAD_PITCH_OBSTACLE
        # if the cmd turn on the right
        if data.angular.z < 0:
            yaw_value = self.HEAD_YAW_OBSTACLE_RIGHT
        # if the cmd turn on the left
        elif data.angular.z > 0:
            yaw_value = self.HEAD_YAW_OBSTACLE_LEFT
        else:
            yaw_value = 0.0

        self.publish_head_control(pitch_value, yaw_value)

    def publish_head_control(self, pitch, yaw):
        self.pt.positions = []

        self.jt.header.stamp = rospy.Time.now() + rospy.Duration(1)
        self.pt.positions.append(pitch)
        self.pt.positions.append(yaw)
        self.pt.time_from_start = rospy.Duration(1)
        self.head_control.publish(self.jt)


if __name__ == "__main__":
    rospy.init_node('move_head')

    headFix = HeadFix()
    headFix.fix_head()
    rospy.spin()
