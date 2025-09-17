# 💻 Development Environment Setup
This page will walk you through how to set up VSCode to connect to the Docker image and run ROS code. You are free to use any IDE you wish, but VSCode is (in our experience) the easiest way to achieve what we need for this class.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
    - ✅ **Ubuntu 22.04 LTS** installed
    - ✅ **Python 3.8+** installed
    - ✅ **ROS 2 Humble** installed via the provided docker image

---

## 🚀 **VS Code (Recommended)**
1. Visit [code.visualstudio.com/download](https://code.visualstudio.com/download) and download the installer.
2. Go to the extensions screen on the left side of your screen (Or hit Ctrl+Shift+X) and install the [Remote Development package](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack), the [Container Tools package](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-containers), the [Docker package](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker), the [Python package](https://marketplace.visualstudio.com/items?itemName=ms-python.python) and the [Python Debugger package](https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy). We also recommend installing a python linter (i.e. Pylance, RUFF) to help with debugging.
3. With WSL open, click the blue button in the bottom left of VSCode and select "Connect to WSL". VSCode should look like it is closing and reopening.
4. After you've connected VSCode to WSL, open a new terminal in VSCode using the Terminal menu on the top on your screen (or Ctrl+Shfit+`). You can check to make sure the connection worked by making sure the username displayed in the terminal matches what's displayed in WSL.
5. Run the docker compose command from the last seciton to verify that VSCode can properly connect to the Docker.

---

## 🐍 **PyCharm (Alternative)**

### **Install PyCharm**
```bash
# Install PyCharm Community Edition
sudo snap install pycharm-community --classic

# Or download from JetBrains website
# https://www.jetbrains.com/pycharm/download/
```

### **Configure Python Interpreter**
1. **Open PyCharm**
2. **File → Settings → Project → Python Interpreter**
3. **Add Interpreter → System Interpreter**
4. **Select**: `/usr/bin/python3`

### **Install Plugins**
- **ROS**: For ROS 2 integration
- **Git Integration**: For version control
- **Markdown**: For documentation
- **Python**: Enhanced Python support

---

### **Cloud Development**
- **GitHub Codespaces**: Cloud development environment
- **GitPod**: Online IDE for GitHub repositories
- **VS Code Online**: Web-based VS Code

---

## 🔍 **Troubleshooting Common Issues**

### **VS Code Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Python not found** | Check interpreter path in settings |
| **Extensions not working** | Restart VS Code, check installation |
| **ROS commands not found** | Source ROS environment in terminal |

### **Environment Issues**
| **Problem** | **Solution** |
|-------------|--------------|
| **Virtual env not activated** | Run `source ~/robotics_env/bin/activate` |
| **Packages not found** | Check pip installation, verify environment |
| **ROS topics not visible** | Check ROS_DOMAIN_ID, source setup.bash |

---

## 🆘 **Getting Help**

### **Development Resources**
- **VS Code Docs**: [code.visualstudio.com/docs](https://code.visualstudio.com/docs)
- **PyCharm Docs**: [jetbrains.com/help/pycharm](https://www.jetbrains.com/help/pycharm/)
- **Python Docs**: [docs.python.org](https://docs.python.org/)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
