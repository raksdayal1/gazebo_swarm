gazebo_swarm

Instructions
1. Create a ros catkin workspace using the following commands. cd into your projects dir (e.q /home/user/my_projects)    
            $ mkdir -p ros_ws/src
            $ cd ..
            $ catkin_make

2. clone the gazebo_swarm package into the src directory
          $ cd src
          $ git clone https://github.com/raksdayal1/gazebo_swarm.git
          $ cd ..
          $ catkin_make

3. Set the ros and gazebo environments
          $ cd ros_ws
          $ source devel/setup.bash
          $ cd ros_ws/src/gazebo_swarm
          $ source setenv.sh

4. Launch gazebo in the same terminal
          $ cd ros_ws
          $ roslaunch gazebo_swarm skyhunter.launch

5. Launch the controller in a second terminal
          $ cd ros_ws
          $ rosrun gazebo_swarm ControlRos.py
