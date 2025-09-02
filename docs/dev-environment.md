# 💻 Development Environment Setup

<div align="center">

**Configure your IDE and development tools for robotics programming**

*Set up VS Code, PyCharm, or other IDEs for efficient ROS 2 and Python development*

</div>

---

## 🎯 **Overview**

This guide helps you set up a professional development environment for robotics programming. You'll learn to configure IDEs, install useful extensions, and set up debugging tools for ROS 2 development.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
- ✅ **Ubuntu 20.04 LTS or 22.04 LTS** installed
- ✅ **Python 3.8+** installed
- ✅ **ROS 2 Humble** installed
- ✅ **Basic terminal knowledge**

---

## 🚀 **VS Code (Recommended)**

### **Install VS Code**
```bash
# Install VS Code
sudo snap install code --classic

# Or download from Microsoft website
# https://code.visualstudio.com/download
```

### **Essential Extensions**
```bash
# Install Python extension
code --install-extension ms-python.python

# Install ROS extension
code --install-extension ms-iot.vscode-ros

# Install C++ extension
code --install-extension ms-vscode.cpptools

# Install Git extension
code --install-extension eamodio.gitlens

# Install Markdown extension
code --install-extension yzhang.markdown-all-in-one

# Install Python Indent
code --install-extension kevinrose.python-indent

# Install Bracket Pair Colorizer
code --install-extension coenraads.bracket-pair-colorizer-2
```

### **VS Code Settings**
```json
// .vscode/settings.json
{
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "python.formatting.provider": "black",
    "python.formatting.blackArgs": ["--line-length", "88"],
    "editor.formatOnSave": true,
    "editor.rulers": [88],
    "files.autoSave": "onFocusChange",
    "python.terminal.activateEnvironment": true
}
```

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

## 🔧 **Terminal Setup**

### **Install Useful Terminal Tools**
```bash
# Install Oh My Zsh (better terminal)
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Install useful packages
sudo apt install htop tree ncdu

# Install tmux for terminal multiplexing
sudo apt install tmux

# Install fzf for fuzzy finding
sudo apt install fzf
```

### **Configure Zsh**
```bash
# Edit .zshrc
nano ~/.zshrc

# Add these lines:
export PATH="$HOME/.local/bin:$PATH"
export ROS_DOMAIN_ID=0
alias ll='ls -la'
alias ..='cd ..'
alias ...='cd ../..'
alias gs='git status'
alias gp='git pull'
alias gc='git commit -m'
```

---

## 📦 **Python Environment Setup**

### **Create Virtual Environment**
```bash
# Create robotics environment
python3 -m venv ~/robotics_env

# Activate environment
source ~/robotics_env/bin/activate

# Install essential packages
pip install numpy matplotlib scipy pandas jupyter

# Install development tools
pip install black pylint pytest ipython

# Install ROS 2 Python client
sudo apt install python3-rclpy
```

### **Jupyter Notebook Setup**
```bash
# Install Jupyter
pip install jupyter notebook

# Install ROS 2 Jupyter extension
pip install ros2-jupyter

# Start Jupyter
jupyter notebook
```

---

## 🐛 **Debugging Setup**

### **VS Code Debugging**
```json
// .vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "cwd": "${workspaceFolder}"
        },
        {
            "name": "ROS 2 Node",
            "type": "python",
            "request": "launch",
            "program": "${workspaceFolder}/src/my_package/my_package/my_node.py",
            "console": "integratedTerminal",
            "cwd": "${workspaceFolder}",
            "env": {
                "ROS_DOMAIN_ID": "0"
            }
        }
    ]
}
```

### **Python Debugging**
```python
# Add breakpoints in your code
import pdb; pdb.set_trace()

# Or use VS Code breakpoints (click left of line numbers)

# Debug with print statements
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)
logger.debug(f"Variable value: {variable}")
```

---

## 🔍 **Code Quality Tools**

### **Black (Code Formatter)**
```bash
# Install Black
pip install black

# Format single file
black filename.py

# Format entire directory
black .

# VS Code integration (already configured above)
```

### **Pylint (Code Linter)**
```bash
# Install Pylint
pip install pylint

# Lint single file
pylint filename.py

# Create pylint configuration
pylint --generate-rcfile > .pylintrc
```

### **Pre-commit Hooks**
```bash
# Install pre-commit
pip install pre-commit

# Create .pre-commit-config.yaml
cat > .pre-commit-config.yaml << EOF
repos:
  - repo: https://github.com/psf/black
    rev: 22.3.0
    hooks:
      - id: black
        language_version: python3
  - repo: https://github.com/pycqa/pylint
    rev: v2.15.0
    hooks:
      - id: pylint
EOF

# Install hooks
pre-commit install
```

---

## 📚 **Project Structure**

### **Recommended Structure**
```
my_robotics_project/
├── .vscode/                 # VS Code settings
├── .gitignore              # Git ignore file
├── README.md               # Project documentation
├── requirements.txt        # Python dependencies
├── setup.py               # Package setup
├── src/                   # Source code
│   └── my_package/
│       ├── __init__.py
│       ├── my_node.py
│       └── utils.py
├── launch/                 # Launch files
│   └── my_launch.py
├── config/                 # Configuration files
│   └── params.yaml
├── test/                   # Tests
│   └── test_my_node.py
└── docs/                   # Documentation
    └── README.md
```

### **Create Project Template**
```bash
# Create project structure
mkdir -p my_robotics_project/{src,launch,config,test,docs,.vscode}
cd my_robotics_project

# Create essential files
touch README.md requirements.txt setup.py .gitignore
touch src/__init__.py launch/__init__.py config/__init__.py

# Initialize Git
git init
git add .
git commit -m "Initial project structure"
```

---

## 🚀 **ROS 2 Development Setup**

### **Create ROS 2 Package**
```bash
# Navigate to workspace
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_package

# Create C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package

# Build workspace
cd ~/ros2_ws
colcon build
```

### **VS Code ROS 2 Integration**
```json
// .vscode/settings.json
{
    "ros.distro": "humble",
    "ros.workspace": "/home/username/ros2_ws",
    "python.defaultInterpreterPath": "/home/username/robotics_env/bin/python"
}
```

---

## 🔧 **Useful Development Scripts**

### **Build Script**
```bash
#!/bin/bash
# build.sh
echo "Building ROS 2 workspace..."

# Clean build
rm -rf build/ install/ log/

# Build with specific packages
colcon build --packages-select my_package

# Source workspace
source install/setup.bash

echo "Build complete!"
```

### **Run Script**
```bash
#!/bin/bash
# run.sh
echo "Running ROS 2 node..."

# Source workspace
source install/setup.bash

# Run node
ros2 run my_package my_node
```

### **Make Scripts Executable**
```bash
chmod +x build.sh run.sh
```

---

## 📱 **Mobile Development**

### **Remote Development**
```bash
# Install VS Code Server
code --install-extension ms-vscode-remote.remote-ssh

# Connect to remote machine
# Ctrl+Shift+P → Remote-SSH: Connect to Host
```

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

## ✅ **Verification Checklist**

- [ ] VS Code installed and configured
- [ ] Essential extensions installed
- [ ] Python virtual environment created
- [ ] ROS 2 integration working
- [ ] Debugging configuration set up
- [ ] Code quality tools installed
- [ ] Project structure created
- [ ] Git repository initialized

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

## 🚀 **Next Steps**

After setting up your development environment:

1. **Practice with VS Code** and Python
2. **Create your first ROS 2 package**
3. **Set up debugging** for your nodes
4. **Start Week 3 lab**: See [Week 3 Lab](labs/week-03.md)

---

<div align="center">

**Ready to code efficiently? Let's start developing! 💻**

[🐍 Python Basics](python-basics.md){ .md-button }
[🤖 ROS Setup](ros-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
