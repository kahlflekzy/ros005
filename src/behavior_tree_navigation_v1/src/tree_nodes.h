#include <behavior_tree_navigation_v1/MoveRobotAction.h>
#include <behavior_tree_navigation_v1/MoveRobotFeedback.h>
#include <behavior_tree_navigation_v1/TasksListenerAction.h>
#include <behavior_tree_navigation_v1/TasksListenerGoal.h>
#include <behavior_tree_navigation_v1/TasksListenerResult.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"

typedef actionlib::SimpleActionClient<behavior_tree_navigation_v1::MoveRobotAction> MoveRobotClient;
typedef actionlib::SimpleActionClient<behavior_tree_navigation_v1::TasksListenerAction> TaskListenerClient;

using namespace BT;
using std::cout;
using std::endl;
using std::string;

class TaskListener : public BT::StatefulActionNode {
    private:
        TaskListenerClient client;
        behavior_tree_navigation_v1::TasksListenerGoal goal;
        int tasks = -1;
        bool flag;
    public:
        TaskListener(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("task_listener", true){
                        ROS_INFO(">>> Waiting for TaskListener Server.");
                        flag = client.waitForServer(ros::Duration(3.0));
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::OutputPort<int>("message")};
        }

        NodeStatus onStart() override {
            if (!flag)
            {   
                ROS_INFO(">>> Failed to connect to task_listener server on time.");
                return NodeStatus::FAILURE;
            }
            goal.start = 1; // start = 1 specifies that server should start listening for goals.
//            ROS_INFO(">>> Sending first goal.");
            client.sendGoal(goal,
                boost::bind(&TaskListener::onActionCompleted, this, _1, _2),
                TaskListenerClient::SimpleActiveCallback(),
                TaskListenerClient::SimpleFeedbackCallback()
            );
//            ROS_INFO(">>> Sent first goal.");
            return NodeStatus::RUNNING;
        }
        
        void onHalted() override {
            // notify the server that the operation has been aborted. NOT_IMPLEMENTED
            client.cancelAllGoals();
            cout<<">>> TasksListener was Halted"<<endl;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && tasks != -1){
                if (tasks == 0)
                {
                    ROS_INFO("Got 0 tasks. Returning FAILURE.");
                    return NodeStatus::FAILURE; // when an empty pose array is sent, stop running tree.
                }
                tasks = -1;
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

          void onActionCompleted(const actionlib::SimpleClientGoalState& state,
                                 const behavior_tree_navigation_v1::TasksListenerResultConstPtr& result){
                // ROS_INFO("Finished in state [%s]", state.toString().c_str());
                tasks = result->tasks;
                setOutput("message", tasks);
            }
};


class ExecuteGoals : public BT::StatefulActionNode
{
    private:
        MoveRobotClient client;
        behavior_tree_navigation_v1::MoveRobotGoal goal;
        int tasks;
        bool flag;

    public:
        ExecuteGoals(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("move_robot", true){
                        initClient();
                    }
        
        static BT::PortsList providedPorts(){
            return {BT::InputPort<int>("message")};
        }

        NodeStatus onStart() override {
            if (!flag){
                ROS_INFO(">>> Failed to connect to move_robot server on time.");
                return NodeStatus::FAILURE;
            }
            getInput("message", tasks);
            ROS_INFO(">>> Got a task with %i points.", tasks);

            goal.id = 0;

            client.sendGoal(goal);

            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                // ROS_INFO(">>> Server has executed goal %i", goal.id);
                goal.id++;
                if (goal.id < tasks)
                {
                    // execute next goal
                    if (client.isServerConnected()){
                        client.sendGoal(goal);
                        return NodeStatus::RUNNING;
                    }else {
                        ROS_INFO("Server down at %i", goal.id);
                        return NodeStatus::SUCCESS; // to be inverted by decorator to FAILURE
                    }
                }
                return NodeStatus::FAILURE; // to be inverted by decorator to SUCCESS
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onHalted() override {
            client.cancelAllGoals();
            ROS_INFO(">>> MoveRobot was Halted");
        }

        void initClient() {
            ROS_INFO(">>> Waiting for MoveRobot Server.");
            flag = client.waitForServer(ros::Duration(3.0));
        }

};
