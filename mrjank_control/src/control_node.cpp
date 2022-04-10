#include <ros/ros.h>
#include <mrjank_control/control.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "control_node");
    ros::NodeHandle nh;
    MrJankInterface controller = MrJankInterface();
    std::vector<std::string> names = {"Rev1","Rev2","Rev3","Rev4","Rev5","Rev6","gripper_servo"};
    std::vector<double> pos,vel;

    ros::Publisher jnt_pub = nh.advertise<sensor_msgs::JointState>("/joint_state",10);

    ros::Rate r(10);
    while(ros::ok()) {
        pos = controller.read_arm_pos();
        vel = controller.read_arm_pos();
        sensor_msgs::JointState state;
        state.header.frame_id = "base_link";
        state.header.stamp = ros::Time::now();
        state.position = pos;
        state.velocity = vel;
        state.name = names; 
        jnt_pub.publish(state);
        r.sleep();
    }

}