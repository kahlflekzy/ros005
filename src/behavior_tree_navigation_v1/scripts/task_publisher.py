#! /usr/bin/python3

from geometry_msgs.msg import PoseArray, Pose
import rospy


class Publisher:
    """"""

    def __init__(self):
        self.pub = rospy.Publisher("/actions_server/poses", PoseArray, queue_size=5)
        self.poses = []
        self.header_frame_id = None
        self.number_of_goals = 0
        self.get_poses()

    def get_poses(self):
        file_path = f"/home/kahlflekzy/PycharmProjects/data/poses/goals_22_06_14_13_55_06.dat"
        with open(file_path) as file:
            goals = file.readlines()
            self.number_of_goals = len(goals)
            for goal in goals:
                self.poses.append(self.goal_to_pose(goal))

    def goal_to_pose(self, goal) -> Pose:
        """"""
        goal = goal.split(";")
        pose = Pose()
        self.header_frame_id = goal.pop(0)

        goal = tuple(map(float, goal))

        pose.position.x = goal[0]
        pose.position.y = goal[1]
        pose.position.z = goal[2]
        pose.orientation.x = goal[3]
        pose.orientation.y = goal[4]
        pose.orientation.z = goal[5]
        pose.orientation.w = goal[6]

        return pose

    def __call__(self, *args, **kwargs):
        rospy.init_node("task_publisher")
        while not rospy.is_shutdown():
            self.prompt()
            try:
                message = self.get_selection()
                self.pub.publish(message)
            except IndexError:
                rospy.loginfo("Wrong Input.")

    def get_selection(self) -> PoseArray:
        """"""
        f = input()
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.header_frame_id
        pose_array.poses = []
        if len(f) != 0 and f != '0':
            fs = map(int, list(f))
            poses = [self.poses[i-1] for i in fs]
            pose_array.poses = poses
        return pose_array

    def prompt(self):
        m = map(str, range(1, len(self.poses)+1))
        message = f"""
                Available for selection {', '.join(m)}\n
                For example, to select pose 1, 2, 3 and 4, type: 1234
                Type just '0' to create an empty PoseArray: 0 
                Note that:
                    1. There is not space in the input.
                    2. Your selection should be within the specified ranges.
                    3. The order of your selection is maintained.\n
                """
        rospy.loginfo(message)


if __name__ == '__main__':
    node = Publisher()
    node()
