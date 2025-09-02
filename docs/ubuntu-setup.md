# 🐧 Ubuntu Setup Guide

<div align="center">

**Complete Ubuntu installation and configuration for robotics development**

*Get your Ubuntu environment ready for ROS 2, Python, and robotics tools*

</div>

---

## 🎯 **Overview**

This guide will help you set up Ubuntu 22.04 LTS for ENME480 robotics development. Ubuntu 22.04 is the recommended operating system for ROS 2 Humble and robotics development.

---

## 💻 **Choose Your Platform**

=== "macOS (Apple Silicon) — UTM VM"

1. **Download UTM** → [https://mac.getutm.app](https://mac.getutm.app)  
2. **Grab Ubuntu 22.04 ARM64** (Desktop or Server): [https://ubuntu.com/download/raspberry-pi](https://ubuntu.com/download/raspberry-pi) (choose **22.04 LTS 64-bit (ARM)**)
3. **Create VM in UTM**  
   - System: **ARM64 (aarch64)**, Memory ≥ **8 GB** (16 GB better), Disk ≥ **60 GB**  
   - Display: **SPICE** (3D acceleration is limited in VMs; Gazebo GUI may be slower)
4. **Install Ubuntu**, then (if you installed Server) add a desktop:
   ```bash
   sudo apt update
   sudo apt install ubuntu-desktop
   sudo reboot
   ```
5. Optional: enable clipboard / directory sharing (UTM docs).

**Refs:** UTM's Ubuntu guide and your reference article (same setup).  
[https://docs.getutm.app/guides/ubuntu/](https://docs.getutm.app/guides/ubuntu/) ; [https://techblog.shippio.io/how-to-run-an-ubuntu-22-04-vm-on-m1-m2-apple-silicon-9554adf4fda1](https://techblog.shippio.io/how-to-run-an-ubuntu-22-04-vm-on-m1-m2-apple-silicon-9554adf4fda1)
{: .thin }

=== "Windows — WSL 2 (Ubuntu 22.04)"

1. **Enable WSL 2** and install **Ubuntu 22.04** from Microsoft Store.
2. Confirm GUI support (**WSLg**) and install your **GPU's vGPU driver** for hardware-accelerated OpenGL:

   * Win11 or Win10 build **19044+** required.
   * Driver links (Intel/AMD/NVIDIA) in Microsoft's WSLg doc.
3. Proceed with ROS 2 / Gazebo steps **inside** the Ubuntu terminal.

**Ref:** Microsoft WSL GUI apps guide.  
[https://learn.microsoft.com/windows/wsl/tutorials/gui-apps](https://learn.microsoft.com/windows/wsl/tutorials/gui-apps)
{: .thin }

=== "Linux / Dual-boot (optional)"

Ubuntu 22.04 LTS native install is fine if you prefer dual-boot. Ensure disk space ≥ **60 GB**.

---

## 💻 **System Requirements**

### **Minimum Requirements**
- **RAM**: 8GB (16GB recommended)
- **Storage**: 60GB free space (100GB recommended)
- **Processor**: 64-bit processor (Intel/AMD) or ARM64 (Apple Silicon)
- **USB**: USB 2.0 or 3.0 port for installation

### **Recommended Setup**
- **RAM**: 16GB or more
- **Storage**: 256GB SSD
- **Processor**: Multi-core processor
- **Graphics**: Dedicated GPU (optional, for simulation)

---

## 📥 **Download Ubuntu**

### **Option 1: Ubuntu Desktop (Recommended)**
1. Go to [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)
2. Download **Ubuntu 22.04 LTS** (latest stable)
3. Choose **64-bit** version

### **Option 2: Ubuntu 22.04 LTS (Recommended)**
- **Direct link**: [Ubuntu 22.04 LTS](https://releases.ubuntu.com/22.04/)
- **Why 22.04**: ROS 2 Humble is officially supported on Ubuntu 22.04 (Tier-1 platform)

---

## 🔧 **Installation Methods**

### **Method 1: Dual Boot (Recommended for beginners)**
- **Pros**: Full performance, native Ubuntu experience
- **Cons**: Requires partitioning, risk of data loss
- **Best for**: Students who want to keep Windows/Mac

### **Method 2: Virtual Machine**
- **Pros**: Safe, no partitioning, easy to remove
- **Cons**: Reduced performance, limited graphics support
- **Best for**: Testing or if you can't dual boot

### **Method 3: WSL2 (Windows only)**
- **Pros**: Good integration with Windows
- **Cons**: Limited graphics support, some compatibility issues
- **Best for**: Windows users who want Linux tools

---

## 🚀 **Dual Boot Installation**

### **Step 1: Prepare Your System**
1. **Backup your data** - Always backup important files
2. **Defragment Windows** (if applicable)
3. **Create free space** - At least 50GB unallocated space

### **Step 2: Create Bootable USB**
1. **Download Rufus** (Windows) or **Balena Etcher** (Mac/Linux)
2. **Insert USB drive** (8GB or larger)
3. **Select Ubuntu ISO** and USB drive
4. **Click Start** and wait for completion

### **Step 3: Boot from USB**
1. **Restart computer** with USB inserted
2. **Enter BIOS/UEFI** (usually F2, F12, or Del)
3. **Change boot order** to USB first
4. **Save and exit**

### **Step 4: Install Ubuntu**
1. **Choose "Install Ubuntu"**
2. **Select language and keyboard layout**
3. **Choose "Install Ubuntu alongside Windows"**
4. **Set partition sizes**:
   - **Ubuntu**: 50-100GB
   - **Swap**: 8-16GB (same as RAM)
   - **Leave Windows partition untouched**
5. **Set timezone and user account**
6. **Wait for installation** (20-30 minutes)

---

## ⚙️ **Post-Installation Setup**

### **Step 1: Update System**
```bash
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
```

### **Step 2: Install Essential Tools**
```bash
# Development tools
sudo apt install build-essential cmake git curl wget

# Python tools
sudo apt install python3-pip python3-venv

# Text editors
sudo apt install code  # VS Code
sudo apt install gedit  # Simple text editor
```

### **Step 3: Configure Python**
```bash
# Create virtual environment for robotics work
python3 -m venv ~/robotics_env
source ~/robotics_env/bin/activate

# Install common packages
pip install numpy matplotlib scipy
```

---

## 🔧 **Common Issues & Solutions**

### **Boot Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Grub not showing** | Boot from USB, run `sudo grub-install` |
| **Windows not in boot menu** | Run `sudo update-grub` |
| **Can't boot Windows** | Use Windows recovery tools |

### **Graphics Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Black screen** | Boot with `nomodeset` kernel parameter |
| **Low resolution** | Install graphics drivers |
| **No display** | Check monitor connections |

### **Network Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **WiFi not working** | Install proprietary drivers |
| **Ethernet not working** | Check cable and drivers |
| **Slow internet** | Update network drivers |

---

## 📱 **Essential Ubuntu Commands**

### **System Management**
```bash
# Update package list
sudo apt update

# Upgrade packages
sudo apt upgrade

# Install package
sudo apt install package_name

# Remove package
sudo apt remove package_name

# Search packages
apt search keyword

# System info
lsb_release -a
uname -a
```

### **File Management**
```bash
# List files
ls -la

# Change directory
cd directory_name

# Create directory
mkdir new_directory

# Copy files
cp source destination

# Move files
mv source destination

# Remove files
rm filename
```

### **Process Management**
```bash
# List processes
ps aux

# Kill process
kill process_id

# System monitor
htop

# Disk usage
df -h
```

---

## 🎨 **Customization (Optional)**

### **Install Additional Software**
```bash
# Media players
sudo apt install vlc

# Image editing
sudo apt install gimp

# Office suite
sudo apt install libreoffice

# Web browsers
sudo apt install firefox
```

### **Customize Desktop**
- **Change wallpaper**: Right-click desktop → Change Background
- **Install themes**: Settings → Appearance
- **Customize dock**: Settings → Dock
- **Add extensions**: Ubuntu Software → Extensions

---

## ✅ **Verification Checklist**

- [ ] Ubuntu boots successfully
- [ ] System updates completed
- [ ] Essential tools installed
- [ ] Python environment configured
- [ ] Network working properly
- [ ] Graphics drivers installed
- [ ] System running smoothly

---

## 🆘 **Getting Help**

### **If Something Goes Wrong**
1. **Check Ubuntu Forums**: [ubuntuforums.org](https://ubuntuforums.org/)
2. **Ask on Piazza**: Course Q&A forum
3. **Office Hours**: Get help from TA or instructor
4. **Ubuntu Documentation**: [help.ubuntu.com](https://help.ubuntu.com/)

### **Emergency Recovery**
- **Boot from USB** and use "Try Ubuntu" mode
- **Use Windows recovery** if dual boot fails
- **Reinstall Ubuntu** as last resort

---

## 🚀 **Next Steps**

After completing Ubuntu setup:

1. **Install ROS 2**: See [ROS Setup Guide](ros-setup.md)
2. **Configure Python**: See [Python Basics](python-basics.md)
3. **Set up Git**: See [Git Basics](git-basics.md)
4. **Install Gazebo**: See [Gazebo Setup](gazebo-setup.md)

---

<div align="center">

**Ready to move on? Let's set up ROS 2 next! 🚀**

[🤖 Install ROS 2](ros-setup.md){ .md-button .md-button--primary }
[🐍 Python Setup](python-basics.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
