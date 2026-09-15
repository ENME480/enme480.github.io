<p class="eyebrow">ENME480 · Wiki</p>

# Ubuntu Setup Guide


## System Requirements

### Minimum Requirements
- **RAM**: 8GB (16GB recommended)
- **Storage**: 60GB free space (100GB recommended)
- **Processor**: 64-bit processor (Intel/AMD) or ARM64 (Apple Silicon)

### Recommended Setup
- **RAM**: 16GB or more
- **Storage**: 256GB SSD
- **Processor**: Multi-core processor
- **Graphics**: Dedicated GPU (optional, for simulation)


## Choose Your Platform

If you are on Windows then you have two choices here:
- A Virtual Machine (VM). This will emulate a second computer running Ubuntu 22.04 inside on your computer. This approach usually works well but can come with a lot of overhead, leading to programms running slowly and crashes.
- Windows Subsystem for Linux (WSL), Microsofts official way of running Linux code on Windows. This approach has much less overhead and runs faster, but can require some extra steps.


=== "macOS (Apple Silicon) — UTM VM"

    ### Step 1: Download UTM

    Download UTM from the official website: <https://mac.getutm.app>

    ### Step 2: Download Ubuntu 22.04 ARM64

    Get Ubuntu 22.04 ARM64 (Desktop or Server): <https://cdimage.ubuntu.mirror.onlime.sl/ubuntu/daily-live/20220417/>  
    *Choose **22.04 LTS 64-bit (ARM)**.*

    ### Step 3: Create New Virtual Machine

    Open UTM and you'll see the welcome screen with options to create a new virtual machine, browse the gallery, or access user guides.

    ![UTM Download](assets/vm_figs/1.webp)

    Choose [Virtualize] then [Linux], choose your downloaded iso image file, and click [Continue] with all of the boxes unchecked.

    ![Ubuntu Download](assets/vm_figs/2.webp)

    ![Create VM](assets/vm_figs/3.webp)

    Click on Browse and select the ISO file you downloaded in Step 2

    ![Choose Virtualization](assets/vm_figs/4.webp)

    On the next screen, leave the memory at 4096 MB and CPU Cores at [Default]. Then specify the amount of space you want to allocate to the virtual machine. It is recommended that you don’t go below around 30GB. Leaving it at the default 64GB is fine, or allocate a higher number if you prefer. Preferred space is around 50GB

    ![Select Linux](assets/vm_figs/5.webp)


    ![Browse ISO](assets/vm_figs/6.webp)

    (Optional) Here you can select a storage location for the VM or just leave it as is. This is to configure a shared directory to make files accessible between macOS and your Ubuntu VM. Click "Browse..." to select a folder.

    ![System Settings](assets/vm_figs/7.webp)

    Once done, enter the details for your VM as you want and press done.

    ### Step 4: Start the VM

    Click the play button to start your virtual machine. You'll see the GRUB boot menu where you can select "Try or Install Ubuntu".

    ![Display Settings](assets/vm_figs/8.webp)


    ![Review Settings](assets/vm_figs/9.png)

    ### Step 5: Ubuntu Installation Welcome

    The below window will be shown and once done, open up "Install Ubuntu 22.04 LTS". The Ubuntu installer will start and show the welcome screen. Select your language and click "Continue".

    ![Start VM](assets/vm_figs/10.png)


    ![Installation Welcome](assets/vm_figs/11.png)

     Choose your keyboard layout. "English (US)" is selected by default. You can test your keyboard in the text field below. (Normally, you can leave it as is and just press continue)

    ![Installation Type](assets/vm_figs/12.png)

    Uncheck the "Download updates while installing" so that you have a faster installation

    ![User Setup](assets/vm_figs/13.png)

     Select "Erase disk and install Ubuntu" since this is a virtual machine. The installer will show a warning about deleting all files.


    ![Installation Progress](assets/vm_figs/14.png)

    ![Installation Complete](assets/vm_figs/15.png)

    ![Ubuntu Login](assets/vm_figs/16.png)

    Enter the details you want and press "Continue". The installer will copy files and install Ubuntu. This process may take several minutes depending on your system performance.

    ![Ubuntu Desktop](assets/vm_figs/17.png)

    Once installation is complete, you'll see the "Installation Complete" screen. Click "Restart Now" to finish the setup.

    ### Step 6: First Boot

    After restart, you'll see the Ubuntu login screen. Enter your username and password to log in.

    ## Troubleshooting

    If your OS doesn't boot up to the welcome screen, restart the VM and press `ESC`, and use your arrow keys to go to "Boot Manager", press "ENTER", go to `ubuntu` and press "ENTER"


    You'll be greeted with the Ubuntu desktop environment with the default jellyfish wallpaper. The dock on the left contains common applications.


    **References:**

    1. [UTM's Ubuntu guide](https://docs.getutm.app/guides/ubuntu/)
    2. [Blog Post](https://techblog.shippio.io/how-to-run-an-ubuntu-22-04-vm-on-m1-m2-apple-silicon-9554adf4fda1)  

=== "Windows — WSL 2 (Ubuntu 22.04)"
    At the moment, getting the Docker image this course uses working in WSL requires that you have an Nvidia GPU. This can be checked by hitting your Windows key and typing *dxdiag*. A popup will appear asking about checking for signed drivers, you can click either option. This should lead you to a window which looks like:
    ![dxdiag screen](assets/nvidia-setup/dxdiag.png)
    Click on the Display tabs on the top to see all available GPUs. If none of them are an NVIDIA GeForce chip you should follow the steps to set up the VM on Windows instead of WSL.

    1. Open **Powershell** (search "powershell" in the Windows menu).  
    2. Install Ubuntu 22.04:

        ```powershell
        wsl --install Ubuntu-22.04
        ```

        This installs the exact distro we use. Using a different Ubuntu version often breaks ROS compatibility. *Note: if you have never used WSL before this command will install some necessary drivers first, then say that it failed to install Ubuntu. If this happens reset your computer and try again, it should work now.*

    3. You should notice that powershell has gona from looking like this:
    ![phsell](assets/nvidia-setup/pshell.png)

    to something like this:
    ![wsl](assets/nvidia-setup/wsl.png)

    This means you are inside WSL. The green part of the lowest line shows your username and domain, while the blue part shows the folder the terminal is currently inside. From here on out, assume that any command we don't explicilty say to run outside WSL should be run from here.


=== "Windows - VM"

    ### Step 1: Download VirtualBox

    Download VirtualBox from the official website: <https://www.virtualbox.org>

    ### Step 2: Download Ubuntu 22.04 ARM64

    Get Ubuntu 22.04 ARM64 (Desktop or Server): <https://releases.ubuntu.com/jammy/>

    *Choose **22.04 LTS 64-bit (AMD)**.*

    ### Step 3: Create New Virtual Machine

    Open UTM and you'll see the welcome screen with options to create a new virtual machine, browse the gallery, or access user guides.

    ![UTM Download](assets/vm_figs/1.webp)

    Choose [Virtualize] then [Linux], choose your downloaded iso image file, and click [Continue] with all of the boxes unchecked.

    ![Ubuntu Download](assets/vm_figs/2.webp)

    ![Create VM](assets/vm_figs/3.webp)

    Click on Browse and select the ISO file you downloaded in Step 2

    ![Choose Virtualization](assets/vm_figs/4.webp)

    On the next screen, leave the memory at 4096 MB and CPU Cores at [Default]. Then specify the amount of space you want to allocate to the virtual machine. It is recommended that you don’t go below around 30GB. Leaving it at the default 64GB is fine, or allocate a higher number if you prefer. Preferred space is around 50GB

    ![Select Linux](assets/vm_figs/5.webp)


    ![Browse ISO](assets/vm_figs/6.webp)

    (Optional) Here you can select a storage location for the VM or just leave it as is. This is to configure a shared directory to make files accessible between macOS and your Ubuntu VM. Click "Browse..." to select a folder.

    ![System Settings](assets/vm_figs/7.webp)

    Once done, enter the details for your VM as you want and press done.

    ### Step 4: Start the VM

    Click the play button to start your virtual machine. You'll see the GRUB boot menu where you can select "Try or Install Ubuntu".

    ![Display Settings](assets/vm_figs/8.webp)


    ![Review Settings](assets/vm_figs/9.png)

    ### Step 11: Ubuntu Installation Welcome

    The below window will be shown and once done, open up "Install Ubuntu 22.04 LTS". The Ubuntu installer will start and show the welcome screen. Select your language and click "Continue".

    ![Start VM](assets/vm_figs/10.png)


    ![Installation Welcome](assets/vm_figs/11.png)

     Choose your keyboard layout. "English (US)" is selected by default. You can test your keyboard in the text field below. (Normally, you can leave it as is and just press continue)

    ![Installation Type](assets/vm_figs/12.png)

    Uncheck the "Download updates while installing" so that you have a faster installation

    ![User Setup](assets/vm_figs/13.png)

     Select "Erase disk and install Ubuntu" since this is a virtual machine. The installer will show a warning about deleting all files.


    ![Installation Progress](assets/vm_figs/14.png)

    ![Installation Complete](assets/vm_figs/15.png)

    ![Ubuntu Login](assets/vm_figs/16.png)

    Enter the details you want and press "Continue". The installer will copy files and install Ubuntu. This process may take several minutes depending on your system performance.

    ![Ubuntu Desktop](assets/vm_figs/17.png)

    Once installation is complete, you'll see the "Installation Complete" screen. Click "Restart Now" to finish the setup.

    ### Step 6: First Boot

    After restart, you'll see the Ubuntu login screen. Enter your username and password to log in.

    ## Troubleshooting

    If your OS doesn't boot up to the welcome screen, restart the VM and press `ESC`, and use your arrow keys to go to "Boot Manager", press "ENTER", go to `ubuntu` and press "ENTER"


    You'll be greeted with the Ubuntu desktop environment with the default jellyfish wallpaper. The dock on the left contains common applications.


    **References:**

    1. [UTM's Ubuntu guide](https://docs.getutm.app/guides/ubuntu/)
    2. [Blog Post](https://techblog.shippio.io/how-to-run-an-ubuntu-22-04-vm-on-m1-m2-apple-silicon-9554adf4fda1)  


=== "Linux / Dual-boot (optional)"

    Ubuntu 22.04 LTS native install is fine if you prefer dual-boot. Ensure disk space ≥ **60 GB**.

## Post-Installation Setup

Open up Terminal using `Ctrl + Alt + T` or from the menu on the bottom left and selecting it.

### Step 1: Update System
```bash
sudo apt update && sudo apt upgrade -y
```

### Step 2: Install Essential Tools

Run these one block at a time and **watch for errors**. A failure part way
through does not stop the later commands, so it is easy to miss one scrolling
past. [Step 4](#step-4-check-your-install) checks the result, so run that when
you are done either way.

```bash
# Tools we need in order to fetch everything else.
# curl has to be installed before anything tries to use it.
sudo apt update
sudo apt install -y apt-transport-https ca-certificates curl gnupg software-properties-common
```

```bash
# Add Docker's package signing key, so apt trusts what it downloads
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
sudo chmod a+r /usr/share/keyrings/docker-archive-keyring.gpg
```

```bash
# Tell apt where to find Docker, then reload the package lists
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
```

```bash
# Development tools
sudo apt install -y build-essential cmake git wget

# Python tools
sudo apt install -y python3-pip python3-venv python-is-python3
```

```bash
# Docker itself
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

!!! warning "Two package names to avoid"
    **Do not run `sudo apt install docker`.** There is an unrelated Ubuntu
    package called `docker` which is a WindowMaker dock applet. It installs
    cleanly, does nothing useful, and makes it look like Docker is installed
    when it is not.

    **Do not use a `docker-compose*` wildcard.** It matches both
    `docker-compose-v2` from Ubuntu and `docker-compose-plugin` from Docker.
    Both ship the same file, so dpkg aborts with
    `trying to overwrite '/usr/libexec/docker/cli-plugins/docker-compose'` and
    the rest of the install is skipped.

    The single command above installs the right things. Compose is included as
    `docker compose` (a subcommand, no hyphen).

### Step 3: Configure Docker to Run as Non-Root User

By default Docker can only be run as an admin, which will cause it to throw lots of random, hard to diagnose errors. These steps will set up Docker so it can be run by anyone. Docker is *supposed* to run these steps manually, but sometimes doesn't so you may see some random warnings or errors about groups already existing or users already being in the group. This is totally fine, we want to rerun these commands to make sure the entire process worked.

Create the docker group if it does not exist:
```bash
sudo groupadd docker
```
Add your user to the docker group:
```bash
sudo usermod -aG docker $USER
```
Log in to the new docker group (to avoid having to log out and log in again; but if not enough, try to reboot):
```bash
newgrp docker
```
Check if Docker can be run without root:
```bash
docker run hello-world
```
This should download a small program and print a short message confirming that Docker works. If it doesn't, reboot using the command:
```bash
reboot
```

### Step 4: Check your install

apt does not stop when one command fails. If something broke earlier, the
commands after it still ran, printed hundreds of lines, and looked like they
worked. This checks what actually ended up on your machine.

Run this one command:

```bash
curl -fsSL https://enme480.github.io/assets/check_setup.sh | bash
```

It only reads. It installs nothing and changes nothing. You can
[read it first](assets/check_setup.sh) if you like.

Every line should say `OK`:

```text
ENME480 setup check

Build tools
  OK   curl
  OK   wget
  ...

Docker
  OK   docker is real Docker, not wmdocker
  OK   docker compose v2
  ...

All 16 checks passed. Your setup is complete.
```

If any line says `FAIL`, fix it using the Repair section below before going on.
The Docker image will not build otherwise.

!!! note "If the command itself fails with `curl: command not found`"
    That is the answer — `curl` never installed. Go back and run the first block
    of [Step 2](#step-2-install-essential-tools), then try again.

??? question "What the less obvious checks are for"
    | Check | Why it matters |
    |-------|----------------|
    | `docker is real Docker, not wmdocker` | Ubuntu has a package called `docker` that is a desktop dock applet. If that got installed instead, `docker` exists as a command but nothing works |
    | `docker compose v2` | We use `docker compose` (no hyphen). The older hyphenated `docker-compose` is a different tool |
    | `your user is in the docker group` | Without it every Docker command needs `sudo`, which breaks file ownership inside the container |
    | `no half-installed packages` | Catches an install that died part way through, which is easy to miss in the scrollback |

### Repair: fixing a partly broken install

Skip this unless Step 4 reported a failure.

**If `wmdocker` was installed**, remove it. It is not Docker and it is not used
by anything:

```bash
sudo apt remove -y docker wmdocker
```

**If the install died on `trying to overwrite '/usr/libexec/docker/cli-plugins/docker-compose'`**,
two conflicting Compose packages were pulled in. Keep Docker's, drop Ubuntu's:

```bash
sudo apt remove -y docker-compose-v2 docker-compose
sudo dpkg --configure -a
sudo apt --fix-broken install -y
```

**If you saw `gpg: no valid OpenPGP data found`**, an empty key file was written
before `curl` existed. Delete it — nothing uses it:

```bash
sudo rm -f /etc/apt/keyrings/docker.gpg
```

**Then reinstall the Docker packages properly and tidy up:**

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt autoremove -y
```

Run Step 4 again. If anything still fails, bring the output to office hours or
post it on Piazza rather than guessing.

## ENME480 Docker Installation

### Step 1: Clone the Repo

Open up [MRC's ENME480 GitHub Repo](https://github.com/MarylandRoboticsCenter/ENME480_mrc). You can either download the zip or open up your terminal

```bash
cd 
git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc.git
```

This will download the repository content into your `HOME` directory. Next, build Docker image (run the command from the docker folder). This needs to be done every time the Docker file is changed. Here's the commands to do that:

**For MacOS users**, change Line no. 4 in the docker file `humble-enme480_ur3e.Dockerfile`

```
# BEFORE
FROM osrf/ros:humble-desktop AS humble-mod_desktop

# AFTER
FROM arm64v8/ros:humble AS humble-mod_desktop
```
**Do not do this on anything other than a MAC!** MACs require code that has been compiled in a special way in order to work and this code does not work on other computers!


### Step 2: Build and Run the Docker

**For Everyone**, run

```bash
cd ~/ENME480_mrc/docker/
userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-compose.yml build
```

Once it is successfully built, run the container with:

```bash
docker compose -f humble-enme480_ur3e-compose.yml run --rm enme480_ur3e-docker
```

In the future, some exercises will require you to open multiple terminals in the same Docker image. In order to achieve this, run:

```bash
docker exec -ti <DELETE THIS AND HIT TAB TO AUTOFILL> bash
```
To spawn a new terminal in the already running container. If you repeatedly run the *docker compose* command you will either get errors or create clones of the container which can not talk to one another.

You should see that your name in the terminal has changed from what is was before to *enme480_mrc*. This means you are inside the Docker container and can run ROS code.

#### Troubleshooting
If you are on WSL and the above compose commands throw an error about an invalid or missing XAuthority, run the following commands:
```bash
printf '\ntouch ~/.Xauthority\nexport XAUTHORITY=$HOME/.Xauthority\n' >> ~/.bashrc
source ~/.bashrc
```
This will create a blank file called .Xauthority in your home directory and tell Ubuntu to look for it. The second line simply reload your environment so Ubuntu trakcs the changes. 

### Step 3 (WSL/Native Ubuntu with an NVIDIA GPU ONLY): Configure Docker to run on NVIDIA GPU

!!! important "Check your GPU before continuing"
    This step applies only if your computer has an **NVIDIA GPU**.

    - **Native Ubuntu with only an Intel or AMD GPU:** Skip this entire step. Continue using the standard `humble-enme480_ur3e-compose.yml` configuration from Step 2.
    - **Windows using WSL with only an Intel or AMD GPU:** The course's WSL setup is not supported without an NVIDIA GPU. Return to the Windows installation choices and follow the **Windows - VM** instructions instead.

    Do not install one of the suggested `nvidia-utils` packages when your computer does not have an NVIDIA GPU. Those packages provide NVIDIA utilities; they cannot add NVIDIA GPU support to Intel or AMD hardware.

First, try running:

```bash
nvidia-smi
```

You should get an output which looks something like:

![smiout](assets/nvidia-setup/smiout.png)

*If you do not see an output like this, you either do not have an NVIDIA GPU or its drivers are not configured correctly. Do not continue with the rest of this NVIDIA configuration step. If your computer does have an NVIDIA GPU, ask an instructor or TA for help configuring its drivers before continuing.*

Getting the correct output from `nvidia-smi` means you have an NVIDIA GPU installed in your computer with its drivers properly configured. Now, we will enable the GPU within Docker to speed up our simulations. First, run the following commands:

```bash
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
```

This will install the Nvidia container toolkit which allows Docker to use your GPU. With the container toolkit installed, we can now configure docker and compose our image:

```bash
echo $'{"runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}}}' > /etc/docker/daemon.json && sudo systemctl restart docker
```

This command will add a line to the settings file to enable running with the Nvidia GPU then resets Docker to reload the configuration.

```bash
cd ~/ENME480_mrc/docker/
userid=$(id -u) groupid=$(id -g) docker compose -f humble-enme480_ur3e-nvidia-compose.yml build
```

Finally, this command will compose and run our image. This is the command you will want to run in order to get into the Docker and use ROS. Once it finishes you should see that the username in the terminal will have changed to "enme480_docker" to let you know that you are in the docker container. From now on, this is the command you will use to launch the docker image.

If you do this step you will launch the container with the command:

```bash
docker compose -f humble-enme480_ur3e-nvidia-compose.yml run --rm enme480_ur3e-docker
```

From now on. The command to connect to a running Docker conatiner (i.e. one you have open in a nother terminal) is still:

```bash
docker exec -ti <hit your tab button> bash
```

You should see that your name in the terminal has changed from what is was before to *enme480_mrc*. This means you are inside the Docker container and can run ROS code.


## Tests for Week 2

To check everything is running, launch the following from within the Docker
image. The demo nodes are already in the image, so there is nothing to install:

```bash
ros2 run demo_nodes_cpp talker
```

This shouuld begin outputting a list of number to the terminal. Open a new terminal, enter the Docker image and run:

```bash
ros2 run demo_nodes_cpp listener
```


This second script should output the messages being sent by the talker.
### Test in New Terminal


```bash
# Open new terminal and run
ros2 --help
# test gazebo, our simulation suite
ign gazebo
```


## Getting Help

### If Something Goes Wrong
1. **Check Ubuntu Forums**: [ubuntuforums.org](https://ubuntuforums.org/)
2. **Ask on Piazza**: Course Q&A forum
3. **Office Hours**: Get help from TA or instructor
4. **Ubuntu Documentation**: [help.ubuntu.com](https://help.ubuntu.com/)

### Emergency Recovery
- **Boot from USB** and use "Try Ubuntu" mode
- **Reinstall Ubuntu** as last resort
