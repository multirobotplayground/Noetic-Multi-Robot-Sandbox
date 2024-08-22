# Table of Contents

- [Table of Contents](#table-of-contents)
  - [Working Environment](#working-environment)
  - [Disclaimer About Docker and ROS](#disclaimer-about-docker-and-ros)
  - [Steps to Install Ubuntu 20.04](#steps-to-install-ubuntu-2004)
  - [Steps to Install ROS 1 Noetic](#steps-to-install-ros-1-noetic)
  - [Steps to Install and Configure Tmux and Git](#steps-to-install-and-configure-tmux-and-git)
  - [Installing Necessary Dependencies](#installing-necessary-dependencies)
  - [Solving TF\_REPEATED\_DATA](#solving-tf_repeated_data)
  - [Install This Workspace](#install-this-workspace)
  - [Next Steps](#next-steps)

## [Working Environment](#working-environment)

This workspace is being developed since 2020 and it was tested on several computers. The last tests occurred on Jul 9, 2024, on an old laptop with a 4th Gen Intel i7 Processor, and it worked on the first attempt. It assumes that you have previous experience installing operating systems on your personal computer and understand tools such as git, curl, bash, and tmux, specifically how to run commands in a terminal.

## [Disclaimer About Docker and ROS](#disclaimer-about-docker-and-ros)

All simulations in this workspace are intended to run on native Ubuntu 20.04. While many students and professionals might prefer to use Docker and install ROS in a container, I believe that the additional system architecture layer introduced by Docker is detrimental to understanding, mainly because ROS itself can be very complicated for the inexperienced explorer, as I’ve explained here. Another reason is that when working with mobile robots in **simulations**, we often need to visualize what is happening. Unfortunately, there is no easy way to make the graphics card driver communicate with the native system if they are different (e.g., a container running Ubuntu 20.04 with a host machine running Ubuntu 24.04 with different glibc libraries), which results in very clunky simulations and **extremely** low fps.

## [Steps to Install Ubuntu 20.04](#steps-to-install-ubuntu-2004)

In summary, you would need to follow these steps:

1. Download Ubuntu 20.04 image from [here](https://www.releases.ubuntu.com/focal/)
2. Use a USB Startup Disk Creator, such as [Rufus](https://rufus.ie/en/) to create a bootable device using the image downloaded from step 1.
3. Restart your computer, boot from the pendrive, and follow the instructions.
4. After finishing the installation, boot in Ubuntu 20.04 and login with your user name and password.
5. Ensure that you have a graphics card good enough to run this workspace packages and that its driver is properly installed.

## [Steps to Install ROS 1 Noetic](#steps-to-install-ros-1-noetic)

In summary, you need to follow these steps:

1. After loggin in Ubuntu, open a terminal.

2. At the terminal install the following package and add an entry pointing to ROS Jazzy repsitories at your sources list with the following commands.

    ```text
    sudo apt install curl
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    ```

3. Install ROS 1 Noetic with the following command. In particullar, I preffer installing the full package for research purposes to reduce the chances of unmet dependencies when testing and working with robots.

    ```text
    sudo apt install ros-noetic-desktop-full
    ```

4. Add an entry at the start of your .bashrc file to make your system be able to see where ROS programs are installed and the resources provided in this workspace.

    ```text
    sed -i '1i export GAZEBO_MODEL_PATH = PATH_TO_MODELS' ~/.bashrc
    sed -i '1i export GAZEBO_RESOURCE_PATH = PATH_TO_RESOURCES' ~/.bashrc
    sed -i '1i source /opt/ros/noetic/setup.bash' ~/.bashrc
    ```

    the starting of your .bashrc file should look something like this

    ```bash
    source /opt/ros/noetic/setup.bash
    export GAZEBO_RESOURCE_PATH = PATH_TO_RESOURCES
    export GAZEBO_MODEL_PATH = PATH_TO_MODELS
    # ~/.bashrc: executed by bash(1) for non-login shells.
    # see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
    # for examples

    # If not running interactively, don't do anything
    case $- in
        *i*) ;;
          *) return;;
    esac
    ```

    In this case, your MUST replace ```PATH_TO_RESORRCES``` with the path of the ```ROS-Noetic-Multi-robot-Sandbox/gazebo_resources/worlds``` and ```PATH_TO_MODELS``` shoudl be ```ROS-Noetic-Multi-robot-Sandbox/gazebo_resources/models``` from this workspace. This allows both, Gazebo 11 and rviz to see the available resources and load the models accordingly.

## [Steps to Install and Configure Tmux and Git](#steps-to-install-and-configure-tmux-and-git)

This project rely on Tmux. Therefore, you **MUST** install it to work with multiple robots and have a clean visualization. I highly recommend to install the theme described here.

1. Install Git hub program to be able to clone and push into remote repositories.

    ```text
    sudo apt install git
    ```

2. Install tmux program with the following command.

    ```text
    sudo apt install tmux
    ```

3. Install tmux plugin manager for a better experience.

    ```text
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
    ```

4. Copy the following configuration into a file named ```~/.tmux.conf```.

    ```bash
    set -g @plugin 'o0th/tmux-nova'

    set -g @nova-nerdfonts true
    set -g @nova-nerdfonts-left 
    set -g @nova-nerdfonts-right 

    set -g @nova-segment-mode "#{?client_prefix,Ω,ω}"
    set -g @nova-segment-mode-colors "#50fa7b #282a36"

    set -g @nova-segment-whoami "#(whoami)@#h"
    set -g @nova-segment-whoami-colors "#50fa7b #282a36"

    set -g @nova-pane "#I#{?pane_in_mode,  #{pane_mode},}  #W"

    set -g @nova-rows 0
    set -g @nova-segments-0-left "mode"
    set -g @nova-segments-0-right "whoami"

    # List of plugins
    set -g @plugin 'tmux-plugins/tpm'
    set -g @plugin 'tmux-plugins/tmux-sensible'
    set -g mouse on

    # Initialize TMUX plugin manager (keep this line at the very bottom of tmux.conf)
    run '~/.tmux/plugins/tpm/tpm'
    ```

5. Open tmux by typing ```tmux``` in a terminal.

6. Enter command mode with ```ctr+b```.
   
7. Install plugins with ```shift+i``` while in command mode.

8. Install a ```patched font``` from [here](https://github.com/ryanoasis/nerd-fonts/releases/download/v3.2.1/AurulentSansMono.zip) and make it default in your terminal. I also recommend turning off the ```Use colors from system theme```.

## [Installing Necessary Dependencies](#installing-necessary-dependencies)

1. Install ```catkin_tools```.

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt update
    sudo apt install python3-catkin-tools
    ```

2. Install ```gmapping```, ```teb_local_planner```, and ```pointcloud-to-laserscan```.

    ```bash
    sudo apt install ros-noetic-gmapping
    sudo apt install ros-noetic-teb-local-planner
    sudo apt install ros-noetic-pointcloud-to-laserscan
    ```

3. Install ```costmap_converter``` standalone node, to do this create a folder in your workspace, clone its repository.

    ```bash
    mkdir -p ~/Documents/dev/CostmapConverterWorkspace/src
    cd ~/Documents/dev/CostmapConverterWorkspace/src
    git clone https://github.com/rst-tu-dortmund/costmap_converter.git && cd ..
    ```

4. Compile and source the workspace, extending the ```ros``` as sourced in your ```.bashrc``` file.

    ```bash
    catkin build && source devel/setup.bash
    ```

## [Solving TF_REPEATED_DATA](#solving-tf_repeated_data)

Ros Noetic has a well known bug, where repeated TF messages flood the terminal if they are detected. For some reason this may happen, specifically in bigger projects with many third party nodes. Unfortunately, even being a well known problem, there is no out-of-the-box solution, and this might prevent you from visualize what is going on in your terminal. To solve this issue you can clone the ```geometry2``` package, commenting the line of code responsible for this and building it yourself.

1. Create a workspace for the ```tf``` packages and clone the project.

    ```bash
    cd && mkdir -p ~/Documents/dev/Geometry2Workspace/src
    cd ~/Documents/dev/Geometry2Workspace/src
    git clone https://github.com/ros/geometry2.git
    ```

2. Open the file ```tf2/src/buffer_core.cpp```.

3. Comment the line ```279``` save and close the file. The line you must comment looks like the following.

    ```bash
    CONSOLE_BRIDGE_logWarn((error_string+" for frame %s (parent %s) at time %lf according to authority %s").c_str(), stripped.child_frame_id.c_str(), stripped.header.frame_id.c_str(), stripped.header.stamp.toSec(), authority.c_str());
    ```

5. Compile and source the workspace, extending the ```costmap_converter``` previously sourced.

    ```bash
    cd ~/Documents/dev/Geometry2Workspace
    catkin build && source devel/setup.bash
    ```

## [Install This Workspace](#install-this-workspace)

1. Clone the repository.

    ```bash
    mkdir -p ~/Documents/dev && cd ~/Documents/dev
    git clone https://github.com/multirobotplayground/ROS-Noetic-Multi-robot-Sandbox.git
    ```

2. Compile and source the workspace, extending the ```geometry2``` previously sourced.

    ```bash
    cd ROS-Noetic-Multi-robot-Sandbox
    catkin build && source devel/setup.bash
    ```
    
## [Next Steps](#next-steps)

  Now your environment should be ready to run the simulations from this repository and also to help you starting your journey. [Click here](usage.md) for the next steps.
