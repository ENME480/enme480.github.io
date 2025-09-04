# ROS2 Docker Setup Guide
In this class we will use ROS2 to enable control of our robots. To assist with this, we have worked with Ivan from MRC to provide a custom docker image with all the necessary drivers built in.
Below, you can find steps to install the docker image on your local machine. 

*These steps vary based on the operating system you are running!*

**Docker Image Download:** https://github.com/MarylandRoboticsCenter/ENME480_mrc

## Windows (10 or later)
1. Open a terminal (it windows and type either *cmd* or *powershell* in the search bar and hit enter)
2. Run the command

        wsl --install Ubuntu-22.04
to install the Ubuntu distribtuion we will use in this class. ROS (the software we use to control the robots) is tightly coupled to specific versions of Ubuntu and it's likely using a version of Ubuntu other than this will lead to compatibility issues. This will take a few minutes to install and then should dorp you into the Ubuntu shell automatically, but yu can type "wsl" to enter the shell if it doens't.
   
4. First, we will make sure our dependencies are in place. Within WSL2 run:
   
        sudo apt update && sudo apt upgrade
   In order to update all system packages (you may need to enter your password)
   
         sudo apt install git python-is-python3 && sudo snap install docker

   To install git (which is how we manage sending code to the class), docker (how we make sure everyone is working in the same environment) and to make Python happier by letting you call it as "python" instead of "python3".
   
         cd ~/ && git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc.git
   To move to the right folder and download the repo containing the docker image we need.

        sudo groupadd docker && sudo usermod -aG docker $USER && newgrp docker
   So that you are able to build and run docker images.
6. Now, we will build our image.

        cd ~/ENME480_mrc/docker && userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
The first part of this command (before the &&) puts you in the folder containing the docker image we want to build, while the second part actually builds our image. This step can take a while, since you have to download a lot of data. If you get a permission error at this step try restarting wsl.


## Ubuntu (22.04 LTS)
