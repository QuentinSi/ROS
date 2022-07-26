#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <interbotix_xs_msgs/JointSingleCommand.h>
#include <interbotix_xs_msgs/JointGroupCommand.h>
#include <interbotix_xs_msgs/JointTrajectoryCommand.h>
#include <ros/time.h>
#include <cmath>
#include <string>
#include <std_srvs/Empty.h>
#include <pckg/move_pos_traj.h>
#include <pckg/move_pos_group.h>
#include <pckg/move_pos_single.h>
#include <pckg/move_pos_simu.h>
#include <pckg/move_pos_simu_rand.h>

using namespace std;
bool moving;

//This is the class with the constructor and the methods for only the simulation Gazebo
class pan_tilt_simu{
private:
    ros::Subscriber sub;
    ros::Publisher arm_pub_pan;
    ros::Publisher arm_pub_tilt;
    ros::NodeHandle *n;
    std_msgs::Float64 angle_pan;
    std_msgs::Float64 angle_tilt;
    std_msgs::Float32 real_angle_pan;
    ros::ServiceServer simu_mov_srv;
    ros::ServiceServer simu_mov_rand_srv;

    int pan_sim=0;
    int tilt_sim=0;
    double rand_pan1=-3.1389;
    double rand_pan2=3.1389;
    double rand_tilt1=-1.57;
    double rand_tilt2=1.57;

public:
    //constructor for gazebo simulation
    pan_tilt_simu(ros::NodeHandle *_n){
        n=_n;
        sub = n->subscribe("wxxms/joint_states", 1000, &pan_tilt_simu::get_position_arm, this);
        arm_pub_pan= n->advertise<std_msgs::Float64>("/wxxms/pan_controller/command", 1000);
        arm_pub_tilt= n->advertise<std_msgs::Float64>("/wxxms/tilt_controller/command", 1000);
        simu_mov_srv= n->advertiseService("move_robot", &pan_tilt_simu::simu_move_pos_srv, this);
        simu_mov_rand_srv= n->advertiseService("move_rand_robot", &pan_tilt_simu::simu_move_pos_rand_srv, this);
    }
    //function to display the data of the robot you want
    void get_position_arm(const sensor_msgs::JointState& state){
        if (state.name.size()== 2 ) {
           for (int i=0;i<2;i++){
           /* ROS_INFO_STREAM("\n RECEIVED JOINT VALUES :"
                            "\n -Joint :" << state.name[i] <<
                            "\n -Position =" << state.position[i] <<
                            "\n -Velocities =" << state.velocity[i] <<
                            //"\n -Acceleration =" <<  state.accelerations[i] <<
                            "\n -Effort =" << state.effort[i] << "\n"); */
          }
        }
    }
    //function to set max and min values ​​for gazebo simulation
    void set_limits(){
        if (angle_pan.data<-3.14) angle_pan.data=-3.139;
        else if (angle_pan.data>3.14) angle_pan.data=3.139;
        if (angle_tilt.data<-1.57) angle_tilt.data=-1.569;
        else if (angle_tilt.data>1.57) angle_tilt.data=1.569;
    }
    bool simu_move_pos_srv( pckg::move_pos_simu::Request &req, pckg::move_pos_simu::Response &resp)
    {
        bool result = set_angle_pan_tilt(req.pos[0], req.pos[1]);
        return result;
    }
    //function to adjust the position of the robot (2 values ​​to enter, the first for the pan motor and the second for the tilt motor)
    bool set_angle_pan_tilt(double fpan, double ftilt){
        if(!moving){
            moving =true;
            angle_pan.data=fpan;
            angle_tilt.data=ftilt;
            set_limits();
            ROS_INFO_STREAM("Pan's value :" << angle_pan << " and Tilt's value: "<< angle_tilt << std::endl);
            arm_pub_pan.publish(angle_pan);
            arm_pub_tilt.publish(angle_tilt);
            moving=false;
        }
        return true;
    }
    //function which makes it possible to vary the position of the robot from its smallest value to its largest value (pan motor)
    void move_auto_pan(){
            arm_pub_pan.publish(angle_pan);
            if(angle_pan.data>=3.139)  pan_sim =1;
            if(angle_pan.data<=-3.139) pan_sim=0;

            if(pan_sim==1) angle_pan.data=angle_pan.data-0.2;
            else if (pan_sim==0) angle_pan.data=angle_pan.data+0.2;

    }
    //function which makes it possible to vary the position of the robot from its smallest value to its largest value (tilt motor)
    void move_auto_tilt(){
        arm_pub_tilt.publish(angle_tilt);
        if(angle_tilt.data>=1.569)  tilt_sim =1;
        if(angle_tilt.data<=-1.569) tilt_sim=0;

        if(tilt_sim==1) angle_tilt.data=angle_tilt.data-0.2;
        else if (tilt_sim==0) angle_tilt.data=angle_tilt.data+0.2;
    }
    bool simu_move_pos_rand_srv( pckg::move_pos_simu_rand::Request &req, pckg::move_pos_simu_rand::Response &resp)
    {
        bool result = random_position();
        return result;
    }
    bool random_position(){
        if(!moving){
            moving=true;
            srand (static_cast <unsigned> (time(0)));
            angle_pan.data=rand_pan1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(rand_pan2-rand_pan1)));
            angle_tilt.data=rand_tilt1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(rand_tilt2-rand_tilt1)));
            set_limits();
            ROS_INFO_STREAM("Pan's value :" << angle_pan << " and Tilt's value: "<< angle_tilt << std::endl);
            arm_pub_pan.publish(angle_pan);
            arm_pub_tilt.publish(angle_tilt);
            moving=false;
        }
        return true;
    }
};

//This is the class for only the simulation RVIZ and the real robot WidowX XM430 with its own constructor and methods
class pan_tilt_real{
private:
    ros::Subscriber sub;
    ros::Publisher real_arm_single;
    ros::Publisher real_arm_group;
    ros::Publisher real_arm_traj;
    ros::ServiceServer move_pos_service;
    ros::ServiceServer move_pos_service_group;
    ros::ServiceServer move_pos_service_single;

    ros::NodeHandle *n;

    interbotix_xs_msgs::JointSingleCommand msg_single;
    interbotix_xs_msgs::JointGroupCommand msg_group;
    interbotix_xs_msgs::JointTrajectoryCommand msg_traj;

    int pan =1;
    int tilt=1;
    double pan_pos;
    double tilt_pos;


public:
    //The constructor with the subscriber and publisher
    pan_tilt_real(ros::NodeHandle *_n){
        n = _n;
        sub = n->subscribe("wxxms/joint_states", 1000, &pan_tilt_real::get_position_arm, this);
        real_arm_single= n->advertise<interbotix_xs_msgs::JointSingleCommand>("/wxxms/commands/joint_single", 1000);
        real_arm_group= n->advertise<interbotix_xs_msgs::JointGroupCommand>("/wxxms/commands/joint_group", 1000);
        real_arm_traj=n->advertise<interbotix_xs_msgs::JointTrajectoryCommand>("/wxxms/commands/joint_trajectory",1000);
        move_pos_service = n->advertiseService("move_pos_traj", &pan_tilt_real::move_pos_srv, this);
        move_pos_service_single = n->advertiseService("move_pos_single", &pan_tilt_real::move_pos_single_srv, this);
        move_pos_service_group = n->advertiseService("move_pos_group", &pan_tilt_real::move_pos_group_srv, this);
    }

    //function to display the data of the robot you want
    void get_position_arm(const sensor_msgs::JointState& state){
        if (state.name.size()== 2 ) {
           /*for (int i=0;i<2;i++){
            ROS_INFO_STREAM("\n RECEIVED JOINT VALUES :"
                            "\n -Joint :" << state.name[i] <<
                            "\n -Position =" << state.position[i] <<
                            "\n -Velocities =" << state.velocity[i] <<
                            //"\n -Acceleration =" <<  state.accelerations[i] <<
                            "\n -Effort =" << state.effort[i] << "\n");
          }*/
           pan_pos = state.position[0];
           tilt_pos = state.position[1];
        }
    }
    bool move_pos_single_srv( pckg::move_pos_single::Request &req, pckg::move_pos_single::Response &resp)
    {
        bool result = move_real_robot_single(req.name_motor, req.pos);
        return result;
    }
    //function that allows you to adjust the position of the desired motor as well as its position
    bool move_real_robot_single(std::string robot_name, double value){
        if(!moving){
            moving=true;
            std::cout << "Moving the pan/tilt ::: " << std::endl;

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
            ROS_INFO_STREAM("Name :" << msg_single.name << " and its value : " << msg_single.cmd << std::endl );

            real_arm_single.publish(msg_single);
            moving=false;
        }
        return true;
    }

    bool move_pos_group_srv( pckg::move_pos_group::Request &req, pckg::move_pos_group::Response &resp)
    {
        bool result = move_manu_real_robot_group(req.pos[0],req.pos[1]);
        return result;
    }

    //function that allows you to set the position data that will be sent to the physical robot
    bool move_manu_real_robot_group(double val_pan, double val_tilt){
        if (!moving){
            moving = true;
            std::cout << "Moving the pan/tilt ::: " << std::endl;

            msg_group.name="all";
            msg_group.cmd.resize(2);
            msg_group.cmd[0]=val_pan;
            msg_group.cmd[1]=val_tilt;
            if (msg_group.cmd[0]>3.14) msg_group.cmd[0]=3.1388;
            else if(msg_group.cmd[0]<-3.14) msg_group.cmd[0]=-3.1388;
            if(msg_group.cmd[1]>1.57) msg_group.cmd[1]= 1.5689;
            else if (msg_group.cmd[1]<-1.57) msg_group.cmd[1]= -1.5689;

            ROS_INFO_STREAM("Pan's value :" << msg_group.cmd[0]<< " and Tilt's value :" << msg_group.cmd[1] << std::endl);
            real_arm_group.publish(msg_group);
            moving =false;
        }
        return true;
    }
    //function that allows the position of the robot to be varied automatically from its smallest value to its largest value authorized
    void move_auto_real_robot(){
        msg_group.name="all";
        msg_group.cmd.resize(2);
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

    bool move_pos_srv( pckg::move_pos_traj::Request &req, pckg::move_pos_traj::Response &resp)
    {
        bool result = move_pos_traj(req.pos[0], req.pos[1], req.vel);
        return result;
    }

    bool move_pos_traj( double pan, double tilt, double duration)
    {
            ROS_INFO_STREAM("Moving to pan = " << pan << " and tilt = " << tilt << std::endl );
            ROS_INFO_STREAM("From pan= " << pan_pos << " and tilt = " << tilt_pos << std::endl );

        if(!moving)
        {
            moving = true;

            std::cout << "Moving the pan/tilt ::: " << std::endl;

            msg_traj.cmd_type="group";
            msg_traj.name="all";
            msg_traj.traj.header.frame_id="pan_tilt";

            msg_traj.traj.header.stamp=ros::Time::now();

            msg_traj.traj.joint_names.resize(2);
            msg_traj.traj.joint_names[0]="pan";
            msg_traj.traj.joint_names[1]="tilt";

            //set the trajectory
            msg_traj.traj.points.resize(9);

            if (pan>3.14) pan = 3.139;
            else if (pan<-3.14) pan = -3.139;

            if (tilt>1.57) tilt = 1.569;
            else if (tilt<-1.57) tilt = -1.569;

            double init_pan =  pan_pos ;
            double init_tilt= tilt_pos ;

            double inc_pan = fabs(init_pan - pan)/8;
            double inc_tilt = fabs(init_tilt - tilt)/8;


            int sig_pan = 1;
            int sig_tilt = 1;

            if(init_pan < pan)
                sig_pan = 1;
            else
                sig_pan = -1;

            if(init_tilt < tilt)
                sig_tilt = 1;
            else
                sig_tilt = -1;

            int trajpoints = 8;
            double step = duration/trajpoints;
            double t_pan, t_tilt;
            double timep = 0.;

            ROS_INFO_STREAM("Generating a trajectory with inc_pan=" <<inc_pan << " and inc_tilt=" << inc_tilt << std::endl );

            for(unsigned int i=0;i<9;i++){
                msg_traj.traj.points[i].positions.resize(2);
                t_pan = init_pan + ((double)sig_pan * i * inc_pan);
                t_tilt = init_tilt + ((double)sig_tilt * i * inc_tilt);

                //ROS_INFO_STREAM("Pan [" << i << "]=" << t_pan << " and tilt[" << i <<"]= " << t_tilt << std::endl );

                msg_traj.traj.points[i].positions[0]=t_pan;
                msg_traj.traj.points[i].positions[1]=t_tilt;
                timep = step + i* step;
                msg_traj.traj.points[i].time_from_start = ros::Duration(timep);
                ROS_INFO_STREAM("Pan [" << i << "]=" << msg_traj.traj.points[i].positions[0] << " and tilt[" << i <<"]= " << msg_traj.traj.points[i].positions[1] << std::endl );
            }

            real_arm_traj.publish(msg_traj);
            moving = false;
         }
        return true;
    }

};


int main (int argc, char** argv){
    //Initialize the ROS system and become a node
    ros::init(argc, argv, "pan_tilt");
    ros::NodeHandle n;
    //Innitialize an object of the class
    pan_tilt_simu robot_simu(&n);
    pan_tilt_real robot_real(&n);

    moving = false;
    ros::Rate loop_rate(10);
    //loop_rate.sleep();

    while(ros::ok()){
        //robot_simu.move_auto_pan();
        //robot_simu.move_auto_tilt();
        //robot_real.move_auto_real_robot();
        ros::spinOnce();
        loop_rate.sleep();
   }
}
