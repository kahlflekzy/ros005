#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>
#include "tree_nodes.h"
#include <signal.h>

using namespace BT;
using std::string;
using std::cout;
using std::endl;
using std::chrono::milliseconds;

int main(int argc, char** argv){
    ROS_INFO("Initialising BehaviorTree Node");
    ros::init(argc, argv, "behaviortree_node");

    ROS_INFO("Initialised BehaviorTree Node");
    char tmp[256];
    string path;
    getcwd(tmp, 256);
    path = string(tmp) + "/src/behavior_tree_navigation_v1/src/xml/tree.xml";

    if (argc > 1)
    {
        path = string(argv[1]) + "/src/xml/tree.xml";
    }

    ROS_INFO("Tree File: %s", path.c_str());
    
    BehaviorTreeFactory factory;

    factory.registerNodeType<ExecuteGoals>("ExecuteGoals");
    factory.registerNodeType<TaskListener>("TaskListener");

    auto tree = factory.createTreeFromFile(path);
    
    NodeStatus status = NodeStatus::RUNNING;
    ROS_INFO("Starting Tree.");
    
    while (status == NodeStatus::RUNNING or status == NodeStatus::SUCCESS){
        if (ros::ok()){
            status = tree.tickRoot();
            tree.sleep(milliseconds(100)); // recommended
        }else{
            tree.haltTree();
            status = NodeStatus::IDLE;
        }
    }
    if (status == NodeStatus::IDLE)
    {
        cout<<"Halted Tree Execution!"<<endl;
    }else {
        ROS_INFO("Finished Tree Execution!");
    }
    return 0;
}