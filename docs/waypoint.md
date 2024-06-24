To launch the waypoint program :
   
Make sure to run the container as instructed in docs/docker.md
open the container using remote-windows and docker plugin in the vscode by selecting the left-bottom icon, choose Attach to Running Container, and then enter the name of the container, in this case "robot_container".
Open the terminal go into /project directory and source the package environment by :
```sh
cd project && source devel/setup.bash
```
Launch the package by :
```sh
./scripts/main.sh
```
Edit the waypoint that we desire inside project/src/task/scripts/waypoint.txt with the format of x, y, tetha
   
open the terminal in host machine, go into ros_motion_planning workspace and then source the package environment by :
```sh
cd project && source devel/setup.bash
```   
Open RVIZ
```sh   
rviz
```
in RVIZ, open file, select rviz config in ros_motion_planning/src/sim_env/rviz/sim_env.rviz
   
open another terminal of docker container, source the workspace environment again and then run the waypoint node by :
```sh
rosrun task waypoint.py
```