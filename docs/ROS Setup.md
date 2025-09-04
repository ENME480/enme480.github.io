# ROS2 Docker Setup Guide
In this class we will use ROS2 to enable control of our robots. To assist with this, we have worked with Ivan from MRC to provide a custom docker image with all the necessary drivers built in.
Below, you can find steps to install the docker image on your local machine. 

*These steps vary based on the operating system you are running!*

**Docker Image Download:** https://github.com/MarylandRoboticsCenter/ENME480_mrc

## Windows (10 or later)
1. Open a terminal (hit windows key and type either *cmd* or *powershell* in the search bar and hit enter)
2. Run the command

        wsl --install Ubuntu-22.04
to install the Ubuntu distribtuion we will use in this class. ROS (the software we use to control the robots) is tightly coupled to specific versions of Ubuntu and it's likely using a version of Ubuntu other than this will lead to compatibility issues. This will take a few minutes to install and then should dorp you into the Ubuntu shell automatically, but you can type *wsl* to enter the shell if it doens't.
   
4. First, we will make sure our dependencies are in place. Within WSL2 run:
   
        sudo apt update && sudo apt upgrade
   In order to update all system packages (you may need to enter your password)
   
         sudo install -m 0755 -d /etc/apt/keyrings && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg && sudo chmod a+r /etc/apt/keyrings/docker.gpg

        sudo apt update && sudo apt upgrade -y

        sudo apt install -y apt-transport-https ca-certificates curl software-properties-common

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

        echo "deb [signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

        sudo apt update
These commands set up package registries within WSL, which is how Ubuntu knows where to look for packages (apps) we want to install. If you'd like a more detailed breakdown of what each command here does, feel free to ask a TA.

        sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin git python-is-python3 docker
This command will install docker (which we use to standarize everyones ROS installation), git (which we use to sync code across computers) and remaps the name "python3" to "python" to make Ubuntu happier when running our code. 

         cd ~/ && git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc.git
To move to the right folder and download the GitHub repo containing the docker image we need.

        sudo groupadd docker 
        
        sudo usermod -aG docker $USER 
        
        newgrp docker

        sudo systemctl restart docker
So that you are able to build and run docker images. These commands make a gruop who can manage docker images, then add you to it, then resets part of Ubuntu so it recognizes the new group. Once this is done, all the parts are in place to build our docker image.
6. Now, we will build our image.

        cd ~/ENME480_mrc/docker && userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
The first part of this command (before the &&) puts you in the folder containing the docker image we want to build, while the second part actually builds our image. This step can take a while, since you have to download a lot of data. If you get a permission error at this step try restarting wsl.

Next, try running

        nvidia-smi
You should get an output which looks something like

*If you do, follow the next step, if not skip to step 9.*
7. Getting the an output from nvidia-smi means you have a Nvidia GPU installed in your computer with drivers properly configured. In this step, we will enable the GPU within docker to speed up our simulations. First, run the following commands:

        sudo touch /etc/docker/daemon.json
	
	sudo chmod 777 /etc/docker/daemon.json

	curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
          && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
	
	sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
	
	sudo apt-get update
	
        export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
        sudo apt-get install -y \
              nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
              libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
This will install the Nvidia container toolkit which allows Docker to use your GPU.
8. With the container toolkit installed, we can now configure docker and compose our image. 

	echo $'{"runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}}}' > /etc/docker/daemon.json && sudo systemctl restart docker
 This command will add a line to the settings file to enable running with the Nvidia GPU then resets Docker to reload the configuration.

        docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
Finally, this command will compose and run our image. This is the command you will want to run in order to get into the Docker and use ROS. Once it finishes you should see that the username in the terminal will have changed to "enme480_docker" to let you know that you are in the docker container. You can skip step 9 if you've done this, step 9 is for people not running with Nvidia GPUs. From here, you can go to the VSCode setup or continue to set up what ever IDE you'd like to use.
9. If you are not running with an Nvidia GPU you can skip setting up the Nvidia toolkit and instead just run:

        docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
This is the command you will need to run to enter the Docker and use ROS. The next step is to configure what ever IDE you'd like to use. We recommend VSCode for it's Docker integration, but you are free to use any IDE you'd like.

### VSCode Setup
1. Visit [code.visualstudio.com/download](https://code.visualstudio.com/download) and download the installer.
2. Go to the extensions screen on the left side of your screen (Or hit Ctrl+Shift+X) and install the [Remote Development package](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack), the [Container Tools package](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-containers), the [Docker package](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker), the [Python package](https://marketplace.visualstudio.com/items?itemName=ms-python.python) and the [Python Debugger package](https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy). We also recommend installing a python linter (i.e. Pylance, RUFF) to help with debugging.
3. With WSL open, click the blue button in the bottom left of VSCode and select "Connect to WSL". VSCode should look like it is closing and reopening.
4. After you've connected VSCode to WSL, open a new terminal in VSCode using the Terminal menu on the top on your screen (or Ctrl+Shfit+`). You can check to make sure the connection worked by making sure the username displayed in the terminal matches what's displayed in WSL.
5. Run the docker compose command from the last seciton.


## Ubuntu (22.04 LTS)
Assuming you are using a fresh install, start from Step 4 in the Windows section. If you are using an installation you've customized you should check what of the prerequisites are already installed to avoid causing dependency issues with other programs. **Warning: If you are using a snap installation of Docker the step to configure docker to use Nvidia Container Toolkit is different, we strongly recommend using the apt installation.** 

Setting up VSCode is also very similar. The only difference is that if you don't want to download the deb package from the microsoft site you can use either apt or snap to get the package via:

        sudo <apt/snap> install code (or code-insiders)

