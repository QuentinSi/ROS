 # Package for WidowX XM430 Robot Turret

To be able to use this project it is very simple, you need the ROS environment to install on your computer. If you don't know how to click on this link, it's the UPC Barcelona tutorial: [Ros_tuto_upc](https://sir.upc.edu/projects/rostutorials/1-ROS_basic_concepts/index.html)


It is strongly advised to work with git, if you want to think about setting it up.
Once your computer is ready and working I advise you to create a project folder. In the project directory (or the name you gave it), create a new folder WidowX XM430 Robot Turret then in this one another under the name of catkin_ws and another one src. With the commande :
```
$mkdir -p WidowX_XM430_Robot_Turret/catkin_ws/src
```

![folder-large](images/access_path.png)

After this create a git_repos, in this folder, create two folders (ioc and robot_turret).

Now open a terminal (Terminator) install following the previous tutorial. Why use Terminator? Terminator organizes terminals in a grid. It is useful because many terminals are usually needed simultaneously when working with ROS. 
Then using the command cd [path] go to the folder you created (git_repos/ioc). Once you are inside do:
`$ git clone [project git https links]` with this [link](https://gitioc.upc.edu/quentin.simon/ros_internship_widowx-xm430.git)
 
* Open a second terminal and go to the folder git_ repos_robot_turret, you need to know its path. 
* Clone this repos in that folder :  

    -[DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK.git)  
    -[interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core.git)  
    -[interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes.git)  
    -[interbotix_ros_turrets](https://github.com/Interbotix/interbotix_ros_turrets.git)  

*  Change to noetic branch in the interbotix repos :
```
$ cd interbotix_ros_core
$ git checkout noetic
$ cd interbotix_ros_toolboxes
$ git checkout noetic
$ cd interbotix_ros_turrets
$ git checkout noetic
```
* Drop the file CATKIN IGNORE
```
$ cd git-repos # or where you have clone the repos
$ cd interbotix_ros_core
$ rm interbotix_ros_xseries/CATKIN_IGNORE
$ cd ..
$ cd interbotix_ros_toolboxes
$ rm interbotix_xs_toolbox/CATKIN_IGNORE
```

So now, in an other terminator go to the folder (WidowX_XM430_Robot_Turret) and do this : 
```
$cd catkin_ws/src
$ ln -s where_git_repos_are_located/DynamixelSDK/ros/dynamixel_sdk
$ ln -s where_git_repos_are_located/interbotix_ros_core
$ ln -s where_git_repos_are_located/interbotix_ros_toolboxes
$ ln -s where_git_repos_are_located/interbotix_ros_turrets
$ ln -s where_git_ioc_are_located/pckg
```
For example, when I tested my path was:

![command](images/command_ln.png)

For pckg the command was :  
`$ ln -s ~/Escriptori/Stage_exo/tst/intership/ros_internship_widowx-xm430/pckg`  
Or `ln -s /home/users/quentin.simon/Escriptori/Stage_exo/git-repos-tuto/ioc/ros_internship_widowx-xm430/pckg`

Once this is done we can compile this file. Just after finishing the previous commands do:
```
$cd ..
$catkin build
```

If the compilation is successful, then you are ready to use the project.

Here is a diagram of what you should have on your computer before the compilation in blue and once the compilation is done the red folders are added to it(this is an example, so these are the names I decided to put, yours may be different but the structure remains the same):

![diagram](images/diagram_project.png)

## Execution step

Before running the programs, check that you have the ttyDXL port which is the one requested to be able to run the simulation programs, if you do not have this port either you configure it or you change it in the programs of the WidowX_XM430 folder. For me, the easiest way is to use this port.

As soon as the port configuration is done, open the pan_tilt.cpp file in the src folder of the pckg folder which is located in the src folder of catkin_ws of WidowX_XM430_Robot_Turret. This is where you choose whether to move the gazebo simulation robot or not. Choose the right class, the program is commented, read the comments to know which class corresponds to what. If you forget to set the  classes the program will run fine but nothing will happen. Normaly, you can use this code without the modify, you can use thanks to services.

Open a terminal, go to the WidowX_XM430 directory then catkin_ws, once here compile then make:
`$ catkin build `
`$ source devel/setup.bash`

Once this is done if you want to use the simulation made:

`$roslaunch interbotix_xsturret_gazebo xsturret_gazebo.launch robot_model:=wxxms`

It is possible that for the gazebo part you have an error when launching it with roslaunch, if this is the case go to interbotix_ros_turrets then interbotix_ros_turrets then interbotix_xsturret_gazebo, go to the launch folder then comment out the line:

 `<arg name="rviz_frame" value="$(arg rviz_frame)"/>` 
 
Recompile then restart roslaunch and the simulation will start.
Or if you are not in simulation but you want to move the physical robot do:

`roslaunch interbotix_xsturret_control xsturret_control.launch robot_model:=wxxms`

After that open a second terminal, go to the same directory as before, compile the program to be sure you have no problem with catkin build, then do:

`$ source devel/setup.bash`

Then run the program:

`rosrun pckg pan_tilt`

For information, the project mainly works with services, check how to use services on the tutorial to be able to use the code correctly. There are only functions that allow the robot to move alone, which launches directly into the main function of pan_tilt.cpp.

Here is an example run:

![example](images/example.png)

If all goes well, you will see the robot doing what you asked it to do using the pan_tilt.cpp program. Now it's up to you, you can develop, modify or just use this project.
