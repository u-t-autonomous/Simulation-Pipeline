#!/usr/bin/env bash

#colors
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

#installing git
install_git() {
    apt-get install git    
} &>/dev/null

#installing ROS Melodic
install_ros() {

    spinner ros_retriever "Installing ROS Melodic" "Finished Installing ROS"

}

spinner(){
    #adds a spinner while processes are running. stdout generalized to parameters 2 and 3
    "$@" 7 &
    PID=$!
    i=1
    delay='.2'
    sp="/-\|"
    echo -n ' '
    echo -e "${CYAN}$2${NC}"
    while [ -d /proc/$PID ]; do
        sleep "${delay}"
        printf "\b${sp:i++%${#sp}:1}"
    done
        printf "${GREEN}$3${NC}\n"
}

ros_retriever() {
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    apt update
    #full install
    apt install ros-melodic-desktop-full
    #rosdep
    sudo rosdep init
    rosdep update

    
} &>/dev/null
#catkin workspace
full_catkin() {
    #use user input to name their workspace
    echo -e "${CYAN}Name your catkin workspace :: ${NC}"
    read dir

    spinner setup_catkinWS "${RED}Creating catkin workspace :: ~/${dir}" "Catkin Workspace Created: :: ~/${dir}\n"    

}
setup_catkinWS() {

    #create their ws
    mkdir -p ~/${dir}/src
    source /opt/ros/melodic/setup.bash
    #build
    (cd ~/${dir} && catkin_make && source ~/${dir}/devel/setup.bash)
    #permissions
    chmod -R 777 ~/${dir}
    #env variables
    

} &>/dev/null

install_turtlebot() {
    echo -e "${CYAN}Installing the Turtlebot3 and its Dependencies"
    spinner clone_turtlebot "Cloning the Turtlebot repositories" "Finished cloning"     
    spinner install_dependencies "Installing Turtlebot Dependencies" "Installed Turtlebot Dependencies"
    environment_variables
}

clone_turtlebot(){
    #also clones the map_merge
    cd ~/${dir}/src
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    git clone https://github.com/hrnr/m-explore
    chmod -R 777 ~/${dir}
    source /opt/ros/melodic/setup.bash 
    cd ~/${dir}
    catkin_make    
} &>/dev/null

install_dependencies(){
    sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
}

environment_variables(){
    #setup their bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "source  ~/${dir}/devel/setup.bash" >> ~/.bashrc
    echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
}

full_execution() {
    spinner install_git "Installing git" "Installed git"
    install_ros
    full_catkin
    install_turtlebot
}
full_execution
