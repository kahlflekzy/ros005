#! /usr/bin/python3

import behavior_tree_navigation_v1.msg as msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

import actionlib
import glob
import os
import rospy
import sys
import typing


class ActionsServer:
    """
    Creates two servers

    - One for communicating tasks with the TaskListener client\
    - Another for communicating with the ExecuteGoals client\
    """

    def __init__(self, path: str = None, init_pose=True):
        """

        :param path: specifies path to a directory where a file with valid 'init_pose' data can be found
        :param init_pose: specifies whether to publish a message which initialises the robot's pose for AMCL
        """
        self.tasks: typing.List[PoseArray] = []
        self.current_task: PoseArray = None
        self.shutdown = False
        self.goal = GoalStatus()

        self.initial_pose_publisher = None
        self.goal_publisher = None

        self.init_subscribers()
        self.init_publishers()

        self.path = path
        self.init_pose = init_pose
        self.initial_pose = None

        self.initialise_pose()

        self.tasks_listener = actionlib.SimpleActionServer("task_listener", msgs.TasksListenerAction, self.listen,
                                                           False)
        self.execute_goal = actionlib.SimpleActionServer("move_robot", msgs.MoveRobotAction, self.move_robot, False)

        self.tasks_listener.start()
        self.execute_goal.start()

    def listen(self, action: msgs.TasksListenerGoal):
        """"""
        rospy.loginfo("Got task request")
        rate = rospy.Rate(10)
        result = msgs.TasksListenerResult()
        if action.start == 1 and not self.shutdown:
            while len(self.tasks) == 0:
                rate.sleep()
            self.current_task = self.tasks.pop(0)
            result.tasks = len(self.current_task.poses)
        else:
            result.tasks = 0
        # rospy.loginfo(f"Got task with {result.tasks} activities and sending to client.")
        self.tasks_listener.set_succeeded(result)

    def move_robot(self, action: msgs.MoveRobotGoal):
        """
        Get goal ids publish goals and notify client of success status.

        :param action:
        :return:
        """
        rospy.loginfo(f"Got action request with goal.id: {action.id}")

        pose_stamped = self.to_pose_stamped(action)
        self.goal_publisher.publish(pose_stamped)

        feedback = msgs.MoveRobotFeedback()

        while self.goal.status != GoalStatus.SUCCEEDED and not self.shutdown:
            feedback.status = self.goal.status
            self.execute_goal.publish_feedback(feedback)
        self.goal.goal_id.id = ''
        self.goal.status = GoalStatus.PENDING

        self.execute_goal.set_succeeded()

    def to_pose_stamped(self, action):
        pose = self.current_task.poses[action.id]
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.current_task.header.frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose

        return pose_stamped

    def init_subscribers(self):
        """Define topics for subscription"""
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.worker)
        rospy.Subscriber("/actions_server/poses", PoseArray, self.add_task)
        rospy.loginfo("Initialised subscribers.")

    def init_publishers(self):
        """Define publishers."""
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.initial_pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(2)

    def add_task(self, data):
        self.tasks.append(data)

    def __call__(self, *args, **kwargs):
        rospy.on_shutdown(self.handle_shutdown)
        rospy.spin()

    def handle_shutdown(self):
        """"""
        # result = msgs.TasksListenerResult()
        # result.tasks = 0
        # self.tasks_listener.set_succeeded(result)
        rospy.loginfo("Cleaning Up.")
        # del self.tasks_listener
        # del self.execute_goal
        self.shutdown = True

    def initialise_pose(self):
        """
        Creates an initial pose and publishes it if self.init_pose is True.

        :return:
        """
        rospy.loginfo("Getting initial pose")
        file = glob.glob(f"{self.path}/init_pose*.dat")[0]
        with open(file) as file_obj:
            poses = file_obj.readlines()
            pose_str = poses[-1]
            pose = self.to_pose_with_covariance(pose_str)
            self.initial_pose = pose

        if self.initial_pose is not None and self.init_pose:
            rospy.loginfo("Publishing initial pose")
            self.initial_pose_publisher.publish(self.initial_pose)

    def worker(self, goal_array_status):
        """"""
        if len(goal_array_status.status_list) != 0:
            goal = goal_array_status.status_list[0]
            if self.goal.goal_id.id == '' and goal.status != GoalStatus.SUCCEEDED:
                self.goal.goal_id.id = goal.goal_id.id
                self.goal.status = goal.status
                # rospy.loginfo(f"\n2. {self.goal}")
            elif self.goal.goal_id.id == goal.goal_id.id:
                self.goal.status = goal.status
                self.goal.text = goal.text

    @staticmethod
    def to_pose(goal: tuple):
        assert len(goal) == 7

        pose = Pose()

        goal = tuple(map(float, goal))

        pose.position.x = goal[0]
        pose.position.y = goal[1]
        pose.position.z = goal[2]
        pose.orientation.x = goal[3]
        pose.orientation.y = goal[4]
        pose.orientation.z = goal[5]
        pose.orientation.w = goal[6]

        return pose

    @staticmethod
    def to_pose_with_covariance(goal: str):
        """
        Takes our string formatted goal and creates and returns a pose with covariance stamped message

        :param goal:
        :return:
        """
        goal = goal.split(";")
        assert len(goal) == 9

        covariance = goal.pop()

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = goal.pop(0)
        pose.header.stamp = rospy.Time.now()

        pose.pose.pose = ActionsServer.to_pose(tuple(map(float, goal)))
        pose.pose.covariance = list(map(float, covariance.split(',')))

        return pose


if __name__ == '__main__':
    rospy.init_node("actions_server")
    rospy.loginfo("Initialised ActionsServer Node.")

    if len(sys.argv) > 1:
        # roslaunch
        init_base = True if sys.argv[2] == "1" else False
        package_path = sys.argv[1]
    else:
        # rosrun
        package_path = os.getcwd() + "/src/data"
        init_base = True
    dir_path = f"/home/kahlflekzy/PycharmProjects/data/poses"
    server = ActionsServer(path=dir_path, init_pose=init_base)
    server()
