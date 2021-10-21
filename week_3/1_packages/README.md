# ROS Packages

#### What is a ROS Package

Packages: Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.

Software in ROS is organized in packages. A package might contain ROS nodes, a ROS-independent library, a dataset, configuration files, a third-party piece of software, or anything else that's useful module. The goal of these packages it to provide this useful functionality in an easy-to-consume manner so that software can be easily reused. In general, ROS packages follow a "Goldilocks" principle: enough functionality to be useful, but not too much that the package is heavyweight and difficult to use from other software.

## Creating a ROS Package

For a package to be considered a catkin package it must meet a few requirements:

-	The package must contain a catkin compliant `package.xml` file. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...
The package must contain a CMakeLists.txt which uses catkin.
- If it is a catkin metapackage it must have the relevant boilerplate `CMakeLists.txt` file.
-  Each package must have its own folder
This means no nested packages nor multiple packages sharing the same directory.

The simplest possible package might have a structure which looks like this:

	my_package/
	  CMakeLists.txt
	  package.xml
	  

### Creating a Package
This tutorial will demonstrate how to use the `catkin_create_pkg ` script to create a new catkin package. This is by far the easiest way to create a package in ROS.

First change to the source space directory of the catkin workspace (if you don't have a catkin workspace, you please create one):

	# You should have created this in the Creating a Workspace Tutorial
	$ cd ~/catkin_ws/src
	
Now use the `catkin_create_pkg ` script to create a new package called `beginner_tutorials ` which depends on std_msgs, roscpp, and rospy:


	$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
	
This will create a beginner_tutorials folder which contains a [package.xml]() and a `CMakeLists.txt`, which have been partially filled out with the information you gave catkin_create_pkg.

catkin_create_pkg requires that you give it a package_name and optionally a list of dependencies on which that package depends:

	# This is an example, do not try to run this
	# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
	
### Building a catkin workspace and sourcing the setup file


Now you need to build the packages in the catkin workspace:

	
	$ cd ~/catkin_ws
	$ catkin_make

After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under `/opt/ros/$ROSDISTRO_NAME`.

To add the workspace to your ROS environment you need to source the generated setup file:


	$ . ~/catkin_ws/devel/setup.bash
	

## Building a ROS Package

### Using catkin_make

`catkin_make` is a command line tool which adds some convenience to the standard catkin workflow. You can imagine that `catkin_make` combines the calls to cmake and make in the standard CMake workflow.

Usage:

	# In a catkin workspace
	$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
	
### Building Your Package

If you are using this page to build your own code, please also take a look at the later tutorials (C++)/(Python) since you may need to modify CMakeLists.txt.

You should already have a catkin workspace and a new catkin package called beginner_tutorials from the previous tutorial, Creating a Package. Go into the catkin workspace if you are not already there and look in the src folder:

	
	$ cd ~/catkin_ws/
	$ ls src
	>> beginner_tutorials/  CMakeLists.txt

You should see that there is a folder called `beginner_tutorials` which you created with `catkin_create_pkg` in the previous tutorial. We can now build that package using `catkin_make`:

	$ catkin_make
	
You should see a lot of output from cmake and then make, which should be similar to this:
	
	Base path: /home/user/catkin_ws
	Source space: /home/user/catkin_ws/src
	Build space: /home/user/catkin_ws/build
	Devel space: /home/user/catkin_ws/devel
	Install space: /home/user/catkin_ws/install
	####
	#### Running command: "cmake /home/user/catkin_ws/src
	-DCATKIN_DEVEL_PREFIX=/home/user/catkin_ws/devel
	-DCMAKE_INSTALL_PREFIX=/home/user/catkin_ws/install" in "/home/user/catkin_ws/build"
	####
	-- The C compiler identification is GNU 4.2.1
	-- The CXX compiler identification is Clang 4.0.0
	-- Checking whether C compiler has -isysroot
	-- Checking whether C compiler has -isysroot - yes
	-- Checking whether C compiler supports OSX deployment target flag
	-- Checking whether C compiler supports OSX deployment target flag - yes
	-- Check for working C compiler: /usr/bin/gcc
	-- Check for working C compiler: /usr/bin/gcc -- works
	-- Detecting C compiler ABI info
	-- Detecting C compiler ABI info - done
	-- Check for working CXX compiler: /usr/bin/c++
	-- Check for working CXX compiler: /usr/bin/c++ -- works
	-- Detecting CXX compiler ABI info
	-- Detecting CXX compiler ABI info - done
	-- Using CATKIN_DEVEL_PREFIX: /tmp/catkin_ws/devel
	-- Using CMAKE_PREFIX_PATH: /opt/ros/kinetic
	-- This workspace overlays: /opt/ros/kinetic
	-- Found PythonInterp: /usr/bin/python (found version "2.7.1") 
	-- Found PY_em: /usr/lib/python2.7/dist-packages/em.pyc
	-- Found gtest: gtests will be built
	-- catkin 0.5.51
	-- BUILD_SHARED_LIBS is on
	-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	-- ~~  traversing packages in topological order:
	-- ~~  - beginner_tutorials
	-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	-- +++ add_subdirectory(beginner_tutorials)
	-- Configuring done
	-- Generating done
	-- Build files have been written to: /home/user/catkin_ws/build
	####
	#### Running command: "make -j4" in "/home/user/catkin_ws/build"
	####
	
Note that catkin_make first displays what paths it is using for each of the 'spaces'. The spaces are described in the REP128 and by documentation about catkin workspaces on the wiki: catkin/workspaces. The important thing to notice is that because of these default values several folders have been created in your catkin workspace. Take a look with ls:

	$ ls
	
## Contact

For questions, please contact Neil