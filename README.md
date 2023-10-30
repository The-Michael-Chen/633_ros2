# 16.633_ros: The NEET Autonomous Drone Class
This class uses Tello drones to autonomously navigate around a track. The track composes of hoops with april tags that the drones must fly through. This project details a progression of assignments that lead up to the final competition. The assignments can be found in the neet_ass folder as html files. The course covers concepts like optical flow, pinhole camera model, image transformations, and PID controls. This github page is specifically for starting the simulation of the drone inside Gazebo for working with ROS2. 

## Class Package installation instructions:
1. pip install virtualenv
2. create a folder for your class projects
3. python<version> -m venv <virtual-environment-name> (python version 3.8.5)
4. git clone this repository follow instructions on the github page
5. pip install -r requirements.txt

## ROS2 Setup Process: 
1. Download Docker Desktop [https://www.docker.com/products/docker-desktop/]
2. In your terminal, run `docker pull tiryoh/ros-desktop-vnc:foxy`. You should see an image appear inside the Docker Images tab on your Docker Desktop app
3. Navigate to the project directory
4. To create a Docker container run `docker run -it -p 6080:80 -v $(pwd):/home/ubuntu/Desktop/<project_name> --name <project_name> tiryoh/ros2-desktop-vnc:foxy`
5. Open the docker container in your browser by clicking the port in docker desktop
6. When inside the docker container, open a terminal and navigate to the project directory in your virtual desktop
7. Run git pull this repository
8. Navigate to the tello_ros_ws and run `source install/setup.bash`
9. Run `colcon build --symlink-install`
10. Visit this link [https://github.com/clydemcqueen/tello_ros] and follow the instructions under install "tello_ros".
11. To run the simulation, click on "tello_gazebo" on the github repo page and follow the instructions there.
12. With the simulation running, in a separate terminal to the flight_ws and run `source install/setup.bash` and then run `colcon build --symlink install`
13. Run `ros2 run competition drone_client`
14. Now you should see the drone takeoff in a simulated environment

<details>
<summary>Python Issue Log</summary>

1. Could not download April Tags plugins:
   Solution: Downgrade Python to version 3.7 or 3.6 using a virtual environment.

2. Flow Point Drifting:
   - Solution 1: Make the Flow point be closer to the top of the screen because when the drone flies forward, you are pointing downwards.
   - Solution 2: Correct your x, y position using the tags before using the flow point.
   - Solution 3: Make sure the surroundings are not one plain surface but have distinguishable things like lines or stark different colors to place flow points on.

3. Failing to Grab Video Stream:
   - Solution: Investigate the issue with your video stream grabbing process.

</details>
<details>
<summary>How to log</summary>

- Custom Model Import Tutorial:
  - [https://www.youtube.com/watch?v=fwoTLfypIMw](https://www.youtube.com/watch?v=fwoTLfypIMw)
  - [https://classic.gazebosim.org/tutorials?tut=build_model#Step3:AddtothemodelSDFfile](https://classic.gazebosim.org/tutorials?tut=build_model#Step3:AddtothemodelSDFfile)
  
- Drone Topic Input:
  - ROS2 Service Call: `/drone1/tello_action` with TelloAction message: `"{cmd: 'takeoff'}"`
  
- How to Use Custom Python Packages:
  - Resources:
    - [https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html)
    - [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html)
  - Steps:
    1. Create a subfolder in your package.
    2. Add an `__init__.py` Python file.
    3. [https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)
  
- How to Read `sensor_msg.msg` as an OpenCV Image with the `cv_bridge` Package:
  - [https://stackoverflow.com/questions/72690021/how-to-process-a-image-message-with-opencv-from-ros2](https://stackoverflow.com/questions/72690021/how-to-process-a-image-message-with-opencv-from-ros2)
  
- Startup Process for Launching Drone in Simulation:
  - ROS2 Launch Command: `ros2 launch tello_gazebo simple_launch.py`
  
- Simulation Time in Gazebo:
  - [https://ceti.pages.st.inf.tu-dresden.de/robotics/howtos/SimulationSpeed.html](https://ceti.pages.st.inf.tu-dresden.de/robotics/howtos/SimulationSpeed.html)
  
- RC Control on the Drone:
  - RC Control Actions: Forward, Left, Up

</details>

<details>
   
<summary>ROS2 Issue Log</summary>

- August 4, 2023
  - Issue with Optical Flow Going to the Side:
    - April tag was rotated the wrong way.
  - Drone Is Not Spawning.
  - Initialization Steps:
    - Had to `pip install` missing packages:
      - pupil apriltags and transformations.
    - Had to import the models into the Gazebo model library.
  - The Drone Is Acting All Crazy.
  
- August 3, 2023
  - Big Issue: Couldn’t Compile Because It Couldn’t Find the Utils Subfolder.
    - Solution: Run with symlink install.
  - Couldn’t Change the Spawn Point.
    - Solution: Change the XML file for the drone and colcon build the tello description package.
  
- July 29, 2023
  - Issue: You Can’t Use a Model in an SDF World Unless You Put the Model in `~/.gazebo/sdf`.

- July 28, 2023
  - Couldn’t Import Custom Model into Gazebo.
    - Solution: Ensure name in the config file matches.
  - Model Was Humongous in Gazebo.
    - Solution: Use 3D print export feature to change export units to meters.
  - To Open a Custom World:
    - Path to worlds:
      - `/usr/share/gazebo-11/worlds`
    - `gazebo worlds/<world_name>.world`.

- July 27, 2023
  - `tello_ros` Package Drone Not Showing Up.
    - Solution: `pip install transformations`.

- July 23, 2023
  - Logging Out Before Stopping the Container Allows You to Use the Browser, But If You Forget to Log Out, You’re Screwed.

- July 17, 2023
  - Can’t Pull Up the `rqt_graph` Even Though Topics Are Available.
    - Solution: Just Close and Reopen the Terminal.

- July 16, 2023
  - TEMP SOLUTION: Can’t Seem to Get the Port to Open Up Locally.
    - Docker command: `docker create -it -p 6080:80 -v $(pwd):/home/ubuntu/Desktop/yondu_ros --name ros2_gazebo --volumes-from ros2_practice tiryoh/ros2-desktop-vnc:foxy`
  - How to Make an Image Based on a Container.
    - Docker command: `docker commit old_container_name new_image_name:tag`
  - Gazebo Issue.
    - Minimizing screen and retrying or run gazebo with `sudo`.
  - Volume Not Saving Issue.
    - Docker command: `docker run -p 6080:80 --shm-size=512m --volume "C:\Users\walte\OneDrive\OneDrive - The University of Texas at Austin\Documents\Personal Projects\Quad Copter Docker\ROS Programs":/home/ubuntu/Desktop/ROS tiryoh/ros2-desktop-vnc:foxy`
    - Volume needs to be before “tiryoh/ros2-desktop-vnc:foxy”.
    - Don’t use `Ctrl+C` to stop the image.
  - `ros_msgs` or Other Packages Not Working.
    - Solution: Rerun source `install/setup.bash`.

- July 15, 2023
  - Issue Installing Xacro.
    - First run `sudo apt update`.
    - `sudo apt-get install ros-foxy-xacro`.
  
  Docker Command:
  - `docker run -it -p 6080:80 -v $(pwd):/home/ubuntu/Desktop/yondu_ros --name ros2_practice tiryoh/ros2-desktop-vnc:foxy`

</details>

