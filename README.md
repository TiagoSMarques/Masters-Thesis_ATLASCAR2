# Masters-Thesis_ATLASCAR2

<!-- One Paragraph of project description goes here -->
ROS packages to establish the position and orientation of the AtlasCar2 based on inclinometer and GPS data, perform the reconstruction of the road perceived by the SICK LD-MRS sensor and from that extract the road boundaries.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites and installing 

This repository contains only the code developed in the context of the thesis. In order to run the code some additional packages need to be installed.

Install the prerequisites for these additional packages

```
%for flycap
sudo apt-get install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0

%GPS libraries
sudo apt-get install ros-kinetic-gps-common 


%Arduino Libraries
sudo apt-get install ros-kinetic-rosserial-python

%Additional libraries
sudo apt-get install libopenni2-dev 
sudo apt-get install libgeos++-dev

```
After this, download the repository containing the code.

```
cd $ros_workspace/
git clone https://github.com/TiagoSMarques/Masters-Thesis_ATLASCAR2.git
mv  -v Masters-Thesis_ATLASCAR2/* src/ && rmdir Masters-Thesis_ATLASCAR2

```
The dependencies for the code need to be compiled first, so remove the developed packages (free_space_detection,orientation_module and road_reconstruction) from the src folder, and compile the code:

```
mv -t . src/free_space_detection src/orientation_module  src/road_reconstruction
catkin_make

```
Since the compilation order is not defined by the user, the compilations could eventually fail because some packages may depend on others that have not been compiled yet. If this occurs move the package in which the error occurred out of the src/ folder and recompile with the same command. 
In the same way for "permission denied" errors in the configuration files run the following code, replacing the name with the name of the file in question:

```
cd directory_of_the_file/
chmod +x  name.cfg
```

Once the compilation has finished re-add the moved packages to the src/ folder and recompile.

## Setup of the hardware and launching the nodes

In the AtlasCar2, connect the Arduino and GPS (2 ports) USB's, and the Ethernet port (with its IP already configured), and launch the nodes for the drivers for the lasers:

```
roslaunch free_space_detection drivers.launch 
```

Then launch the node to run the inclinometer module, replacing "port_name" with the device name refering to the Arduino USB:

```
rosrun rosserial_python serial_node.py /dev/port_name
```

Finally launch the road reconstruction system, which launches all the nodes needed, and presents the point cloud data in Rviz for visualization:

```
rosrun rosserial_python serial_node.py /dev/port_name
```



### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system



## Built For the ROS environment

* [ROS](hthttp://www.ros.org/about-ros/) - The Robot Operating System

## Authors

* **Tiago Marques** - *Master's Thesis* - [Universidade de Aveiro](https://github.com/TiagoSMarques)

<!-- See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project. -->

<!-- ## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details -->

## Acknowledgments

<!-- * Hat tip to anyone whose code was used
* Inspiration
* etc -->
