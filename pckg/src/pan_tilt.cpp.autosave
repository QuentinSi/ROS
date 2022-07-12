#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "pckg/JointSingleCommand.h"
#include "pckg/JointGroupCommand.h"
#include "pckg/JointTrajectoryCommand.h"
#include <ros/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//This is the class with the constructor and the methods for only the simulation Gazebo
class pan_tilt_simu{
private:
    ros::Subscriber sub;
    ros::Publisher arm_pub_pan;
    ros::Publisher arm_pub_tilt;
    ros::NodeHandle n;
    std_msgs::Float64 angle_pan;
    std_msgs::Float64 angle_tilt;
    std_msgs::Float32 real_angle_pan;
    int pan_sim=0;
    int tilt_sim=0;
    double rand_pan1=-3.1389;
    double rand_pan2=3.1389;
    double rand_tilt1=-1.57;
    double rand_tilt2=1.57;

public:
    //constructor for gazebo simulation
    pan_tilt_simu(){
        sub = n.subscribe("wxxms/joint_states", 1000, &pan_tilt_simu::get_position_arm, this);
        arm_pub_pan= n.advertise<std_msgs::Float64>("/wxxms/pan_controller/command", 1000);
        arm_pub_tilt= n.advertise<std_msgs::Float64>("/wxxms/tilt_controller/command", 1000);
    }
    //function to display the data of the robot you want
    void get_position_arm(const sensor_msgs::JointState& state){
        //ROS_INFO("RECEIVED JOINT VALUES");
        if (state.name.size()== 2 ) {
           for (int i=0;i<2;i++){
            ROS_INFO_STREAM("\n RECEIVED JOINT VALUES :"
                            "\n -Joint :" << state.name[i] <<
                            "\n -Position =" << state.position[i] <<
                            "\n -Velocities =" << state.velocity[i] <<
                            //"\n -Acceleration =" <<  state.accelerations[i] <<
                            "\n -Effort =" << state.effort[i] << "\n");
          }
        }
    }
    //function to set max and min values ​​for gazebo simulation
    void set_limits(){
        if (angle_pan.data<-3.14) angle_pan.data=-3.14;
        else if (angle_pan.data>3.14) angle_pan.data=3.14;
        else if (angle_tilt.data<-1.57) angle_tilt.data=-1.57;
        else if (angle_tilt.data>1.57) angle_tilt.data=1.57;
    }
    //function that allows to put the robot of the gazebo simulation in initial position
    void init_angle(){
        angle_pan.data= 0.0;
        angle_tilt.data= 0.0;
        arm_pub_pan.publish(angle_pan);
        arm_pub_tilt.publish(angle_tilt);
    }
    //function to adjust the position of the robot (2 values ​​to enter, the first for the pan motor and the second for the tilt motor)
    void set_angle_pan_tilt(double fpan, double ftilt){
        angle_pan.data=fpan;
        angle_tilt.data=ftilt;
        set_limits();
        arm_pub_pan.publish(angle_pan);
        arm_pub_tilt.publish(angle_tilt);
    }
    //function which makes it possible to vary the position of the robot from its smallest value to its largest value (pan motor)
    void move_auto_pan(){
            arm_pub_pan.publish(angle_pan);
            if(angle_pan.data>=3.14)  pan_sim =1;
            if(angle_pan.data<=-3.139) pan_sim=0;

            if(pan_sim==1) angle_pan.data=angle_pan.data-0.2;
            else if (pan_sim==0) angle_pan.data=angle_pan.data+0.2;
    }
    //function which makes it possible to vary the position of the robot from its smallest value to its largest value (tilt motor)
    void move_auto_tilt(){
        arm_pub_tilt.publish(angle_tilt);
        if(angle_tilt.data>=1.57)  tilt_sim =1;
        if(angle_tilt.data<=-1.57) tilt_sim=0;

        if(tilt_sim==1) angle_tilt.data=angle_tilt.data-0.2;
        else if (tilt_sim==0) angle_tilt.data=angle_tilt.data+0.2;
    }
    void random_position(){
        srand (static_cast <unsigned> (time(0)));
        angle_pan.data=rand_pan1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(rand_pan2-rand_pan1)));
        angle_tilt.data=rand_tilt1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(rand_tilt2-rand_tilt1)));
        set_limits();
        arm_pub_pan.publish(angle_pan);
        arm_pub_tilt.publish(angle_tilt);

    }
};

//This is the class for only the simulation RVIZ and the real robot WidowX XM430 with its own constructor and methods
class pan_tilt_real{
private:
    ros::Subscriber sub;
    ros::Publisher real_arm_single;
    ros::Publisher real_arm_group;
    ros::Publisher send_velo;
    ros::Publisher real_arm_traj;
    ros::NodeHandle n;
    pckg::JointSingleCommand msg_single;
    pckg::JointGroupCommand msg_group;
    int pan =1;
    int tilt=1;
    sensor_msgs::JointState jointstate;
    trajectory_msgs::JointTrajectory traj;
    pckg::JointTrajectoryCommand msg_traj;
    trajectory_msgs::JointTrajectoryPoint points_n;


public:
    //The constructor with the subscriber and publisher
    pan_tilt_real(){
        sub = n.subscribe("wxxms/joint_states", 1000, &pan_tilt_real::get_position_arm, this);
        real_arm_single= n.advertise<pckg::JointSingleCommand>("/wxxms/commands/joint_single", 1000);
        real_arm_group= n.advertise<pckg::JointGroupCommand>("/wxxms/commands/joint_group", 1000);
        send_velo= n.advertise<sensor_msgs::JointState>("/wxxms/joint_states", 1000);
        real_arm_traj=n.advertise<pckg::JointTrajectoryCommand>("/wxxms/commands/joint_trajectory",1000);
    }
    //function to display the data of the robot you want
    void get_position_arm(const sensor_msgs::JointState& state){
        //ROS_INFO("RECEIVED JOINT VALUES");
        if (state.name.size()== 2 ) {
           for (int i=0;i<2;i++){
            ROS_INFO_STREAM("\n RECEIVED JOINT VALUES :"
                            "\n -Joint :" << state.name[i] <<
                            "\n -Position =" << state.position[i] <<
                            "\n -Velocities =" << state.velocity[i] <<
                            //"\n -Acceleration =" <<  state.accelerations[i] <<
                            "\n -Effort =" << state.effort[i] << "\n");
          }
        }
    }
    //function that allows you to adjust the position of the desired motor as well as its position
    void move_real_robot_single(const char* robot_name, double value){
        msg_single.name = robot_name;
        msg_single.cmd=value;

        if(msg_single.name=="pan"){
            if (msg_single.cmd>3.14) msg_single.cmd=3.1388;
            else if(msg_single.cmd<-3.14) msg_single.cmd=-3.1388;
        }
        if(msg_single.name=="tilt"){
            if (msg_single.cmd>1.57) msg_single.cmd= 1.5689;
            else if (msg_single.cmd<-1.57) msg_single.cmd= -1.5689;

        }
        real_arm_single.publish(msg_single);
    }
    //function that allows you to set the position data that will be sent to the physical robot
    void move_manu_real_robot_group(double val_pan, double val_tilt){
        msg_group.name="all";
        msg_group.cmd.resize(2);
        msg_group.cmd[0]=val_pan;
        msg_group.cmd[1]=val_tilt;
        if (msg_group.cmd[0]>3.14) msg_group.cmd[0]=3.1388;
        else if(msg_group.cmd[0]<-3.14) msg_group.cmd[0]=-3.1388;
        else if (msg_group.cmd[1]>1.57) msg_group.cmd[1]= 1.5689;
        else if (msg_group.cmd[1]<-1.57) msg_group.cmd[1]= -1.5689;
        real_arm_group.publish(msg_group);
    }
    //function that allows the position of the robot to be varied automatically from its smallest value to its largest value authorized
    void move_auto_real_robot(){
        msg_group.name="all";
        msg_group.cmd.resize(2);
        //msg_group.cmd[1]=0.0;
        //msg_group.cmd[0]=0.0;
        if (msg_group.cmd[0]>=3.1388) pan=1;
        else if(msg_group.cmd[0]<= -3.1388) pan=0;

        if (pan==1)msg_group.cmd[0]=msg_group.cmd[0]-0.02;
        else if(pan==0) msg_group.cmd[0]=msg_group.cmd[0]+0.02;

        if (msg_group.cmd[1]>=1.5689) tilt=1;
        else if(msg_group.cmd[1]<= -1.5689) tilt=0;

        if (tilt==1)msg_group.cmd[1]=msg_group.cmd[1]-0.02;
        else if(tilt==0) msg_group.cmd[1]=msg_group.cmd[1]+0.02;
        real_arm_group.publish(msg_group);
    }

    void traject(){
        msg_traj.cmd_type="group";
        msg_traj.name="all";
        msg_traj.traj.header.stamp=ros::Time::now();
        msg_traj.traj.header.frame_id="";
        msg_traj.traj.joint_names.resize(2);
        msg_traj.traj.joint_names[0]="pan";
        msg_traj.traj.joint_names[1]="tilt";

        msg_traj.traj.points.resize(3);
        msg_traj.traj.points[0].positions.resize(2);
        msg_traj.traj.points[0].positions[0]=-1.0;
        msg_traj.traj.points[0].positions[1]=-1.0;
        msg_traj.traj.points[0].time_from_start = ros::Duration(1);
        real_arm_traj.publish(msg_traj);

        int j=0;

        for(int i=1;i<3;i++){

            msg_traj.traj.points[i].positions.resize(2);
            msg_traj.traj.points[i].positions[0]=i+0.5;
            msg_traj.traj.points[i].positions[1]=j+0.5;
            j++;
            msg_traj.traj.points[i].time_from_start =  msg_traj.traj.points[i-1].time_from_start+ros::Duration(3.0);
            real_arm_traj.publish(msg_traj);
        }
    }

};

int main (int argc, char** argv){
    ros::init(argc, argv, "pan_tilt");
    //pan_tilt_simu robot_simu;
    pan_tilt_real robot_real;
    ros::Rate loop_rate(10);

    while(ros::ok()){
        //robot_simu.init_angle();
        //robot_simu.set_angle_pan_tilt(10.0, -11.0);
        //robot_simu.move_auto_pan();
        //robot_simu.move_auto_tilt();
        //robot_simu.random_position();
        //robot_real.move_real_robot_single("tilt", 0.0);
        //robot_real.move_real_robot_single("pan", 1.0);
        robot_real.move_manu_real_robot_group(0.0, 0.0);
        //robot_real.move_auto_real_robot();
        //robot_real.traject();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
