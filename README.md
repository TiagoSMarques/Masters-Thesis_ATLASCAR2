# Masters-Thesis_ATLASCAR2

<!-- One Paragraph of project description goes here -->
ROS packages to establish the position and orientation of the ATLASCAR2 based on inclinometer and GPS data, perform the reconstruction of the road percieved by the SICK LD-MRS sensor and from that extract the road boundaries.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites and installing 

This repository contains only the code developed in the constext of the thesis. In order to run the code some addicional packeges need to be installed. For this a a repository was created...


```
cd $ros_workspace/
git clone ...
mv  -v package_name/* src/ && rmdir package_name
```
Install the prerequitis for these additional packages

```
%for flycap
sudo apt-get install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0

%GPS libraries
sudo apt-get install ros-kinetic-gps-common 

%Additional libraries
sudo apt-get install libopenni2-dev 
sudo apt-get install libgeos++-dev

```

After this build the dependencies by running


```
catkin_make
```
Since the compilation order is not defined by the user, the compilations could eventually fail, because some packages may depend on others that have not been compiled yet. If this occurs move the package in which the error occured out of the src/ folder and recompile with the same comand.
Once the compilation has finished re-add the moved pasckages to the src/ folder and recompile.

After this clone this repo and move its contents to the src folder

```
git clone https://github.com/TiagoSMarques/Masters-Thesis_ATLASCAR2.git
mv  -v Masters-Thesis_ATLASCAR2/* src/ && rmdir Masters-Thesis_ATLASCAR2

```
And compile, if necessary aplying the same precautions explained in the step before.

```
catkin_make
```

<!-- ### Installing

A step by step series of examples that tell you how to get a development env running.

```

```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo -->

## Running the tests

Explain how to run the automated tests for this system

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
