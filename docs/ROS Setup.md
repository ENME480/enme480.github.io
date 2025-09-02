# ROS2 Docker Setup Guide
In this class we will use ROS2 to enable control of our robots. To assist with this, we have worked with Ivan from MRC to provide a custom docker image with all the necessary drivers built in.
Below, you can find steps to install the docker image on your local machine. 

*These steps vary based on the operating system you are running!*

**Docker Image Download:** https://github.com/MarylandRoboticsCenter/ENME480_mrc

## Windows
1. Download the Docker Image
2. Download WSL2 (Windows Subsystem for Linux), running Ubuntu 22.04 (*our code is version locked - do not doesnload 24.04 or things will break!*)
3. Within WSL2 run:
    sudo apt update && sudo apt upgrade
   In order to update all system packages (you may need to enter your password)
     sudo apt install git && sudo snap install docker
   In order to install the relevant packages
     git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc.git
   To download the repo
     docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
5. 


## Ubuntu
