# 🔧 Git & GitHub Basics

<div align="center">

**Essential version control for robotics development**

*Learn Git to manage your lab code, collaborate with teammates, and track your progress*

</div>

---

## 🎯 **Overview**

Git is a version control system that helps you track changes in your code, collaborate with others, and maintain a history of your work. This guide covers the essential Git commands you'll need for ENME480 labs and projects.

---

## 💻 **Prerequisites**

Before starting, ensure you have:
- ✅ **Git installed** on your system
- ✅ **GitHub account** created
- ✅ **Basic terminal knowledge**

---

## 🚀 **Getting Started**

### **Install Git**
```bash
# Ubuntu/Debian
sudo apt install git

# Check installation
git --version
```

### **Configure Git**
```bash
# Set your name and email
git config --global user.name "Your Name"
git config --global user.email "your.email@umd.edu"

# Check configuration
git config --list
```

---

## 📚 **Core Git Concepts**

### **Repository (Repo)**
- **Local repository**: Git repository on your computer
- **Remote repository**: Git repository on GitHub/GitLab
- **Clone**: Copy remote repository to local machine

### **Working Directory**
- **Working directory**: Where you edit files
- **Staging area**: Area where changes are prepared for commit
- **Repository**: Where committed changes are stored

### **Basic Workflow**
1. **Edit files** in working directory
2. **Stage changes** to staging area
3. **Commit changes** to repository
4. **Push changes** to remote repository

---

## 🔧 **Essential Git Commands**

### **Initializing a Repository**
```bash
# Create new repository
git init

# Clone existing repository
git clone https://github.com/username/repository.git

# Check repository status
git status
```

### **Making Changes**
```bash
# Stage all changes
git add .

# Stage specific files
git add filename.py
git add *.py

# Check what's staged
git diff --cached

# Commit changes
git commit -m "Add robot control functions"

# Check commit history
git log
git log --oneline
```

### **Managing Branches**
```bash
# List branches
git branch

# Create new branch
git branch feature-name

# Switch to branch
git checkout feature-name

# Create and switch to new branch
git checkout -b feature-name

# Merge branch
git checkout main
git merge feature-name

# Delete branch
git branch -d feature-name
```

---

## 🌐 **Working with GitHub**

### **Setting up SSH Keys**
```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your.email@umd.edu"

# Start SSH agent
eval "$(ssh-agent -s)"

# Add SSH key
ssh-add ~/.ssh/id_ed25519

# Copy public key to GitHub
cat ~/.ssh/id_ed25519.pub
```

### **Remote Operations**
```bash
# Add remote repository
git remote add origin https://github.com/username/repository.git

# Check remotes
git remote -v

# Push changes
git push origin main

# Pull changes
git pull origin main

# Fetch changes (without merging)
git fetch origin
```

---

## 📝 **Common Workflows**

### **Daily Workflow**
```bash
# Start of day - get latest changes
git pull origin main

# Make changes to files
# ... edit files ...

# Check what changed
git status
git diff

# Stage and commit changes
git add .
git commit -m "Implement joint limit checking"

# Push changes
git push origin main
```

### **Feature Development**
```bash
# Create feature branch
git checkout -b feature/joint-control

# Make changes
# ... implement feature ...

# Commit changes
git add .
git commit -m "Add joint control system"

# Push feature branch
git push origin feature/joint-control

# Create pull request on GitHub
# ... merge on GitHub ...

# Switch back to main
git checkout main
git pull origin main

# Delete feature branch
git branch -d feature/joint-control
```

---

## 🔍 **Useful Git Commands**

### **Viewing History**
```bash
# View commit history
git log --oneline --graph

# View changes in specific commit
git show commit_hash

# View changes in working directory
git diff

# View staged changes
git diff --cached
```

### **Managing Files**
```bash
# Remove file from Git
git rm filename.py

# Rename file
git mv oldname.py newname.py

# Ignore files
echo "*.log" >> .gitignore
echo "build/" >> .gitignore
```

### **Undoing Changes**
```bash
# Undo last commit (keep changes)
git reset --soft HEAD~1

# Undo last commit (discard changes)
git reset --hard HEAD~1

# Undo changes in working directory
git checkout -- filename.py

# Revert specific commit
git revert commit_hash
```

---

## 📱 **GitHub Desktop (Optional)**

### **Why Use GitHub Desktop?**
- **Visual interface** for Git operations
- **Easier for beginners** than command line
- **Built-in merge conflict resolution**
- **Good for simple workflows**

### **Installation**
```bash
# Download from GitHub
# https://desktop.github.com/

# Install and authenticate with your GitHub account
```

---

## 🧪 **Practice Exercises**

### **Exercise 1: Create a Lab Repository**
```bash
# Create directory for lab work
mkdir enme480-labs
cd enme480-labs

# Initialize Git repository
git init

# Create README file
echo "# ENME480 Lab Work" > README.md

# Make first commit
git add README.md
git commit -m "Initial commit: Add README"

# Create GitHub repository and push
git remote add origin https://github.com/username/enme480-labs.git
git push -u origin main
```

### **Exercise 2: Work with Branches**
```bash
# Create feature branch
git checkout -b lab/week1

# Create lab file
echo "# Week 1 Lab" > week1.md

# Commit changes
git add week1.md
git commit -m "Add Week 1 lab notes"

# Push feature branch
git push origin lab/week1

# Switch back to main
git checkout main

# Merge feature branch
git merge lab/week1

# Push merged changes
git push origin main
```

---

## 🔧 **Troubleshooting Common Issues**

### **Merge Conflicts**
```bash
# When you get a merge conflict
git status  # See conflicted files

# Edit conflicted files to resolve conflicts
# Look for <<<<<<<, =======, >>>>>>> markers

# After resolving conflicts
git add .
git commit -m "Resolve merge conflicts"
```

### **Authentication Issues**
```bash
# If you get authentication errors
git config --global credential.helper store

# Or use personal access token
# Generate token on GitHub: Settings → Developer settings → Personal access tokens
```

### **Large Files**
```bash
# If you accidentally commit large files
git filter-branch --tree-filter 'rm -f large_file.dat' HEAD

# Or use Git LFS for large files
git lfs install
git lfs track "*.dat"
```

---

## 📚 **Git Best Practices**

### **Commit Messages**
- **Use present tense**: "Add feature" not "Added feature"
- **Be descriptive**: "Implement joint limit checking" not "Fix bug"
- **Keep it short**: First line under 50 characters
- **Use imperative mood**: "Add", "Fix", "Update", "Remove"

### **Branch Naming**
- **Feature branches**: `feature/description`
- **Bug fixes**: `fix/description`
- **Lab work**: `lab/week1`, `lab/week2`
- **Project work**: `project/final-project`

### **When to Commit**
- **After completing a logical unit** of work
- **Before making major changes** to working code
- **After fixing a bug** or implementing a feature
- **At the end of each lab session**

---

## 🆘 **Getting Help**

### **Git Resources**
- **Git Documentation**: [git-scm.com](https://git-scm.com/doc)
- **GitHub Guides**: [guides.github.com](https://guides.github.com/)
- **Git Cheat Sheet**: [git-scm.com](https://git-scm.com/doc/cheatsheet)

### **Course Support**
- **Piazza**: Ask questions on course forum
- **Office Hours**: Get help from TA or instructor
- **Lab Sessions**: Hands-on help during labs

---

## 🚀 **Next Steps**

After mastering Git basics:

1. **Set up your lab repository** on GitHub
2. **Practice with daily commits** during lab work
3. **Collaborate with teammates** on group projects
4. **Learn advanced Git features** as needed

---

<div align="center">

**Ready to manage your code? Let's set up your lab repository! 📚**

[🐧 Ubuntu Setup](ubuntu-setup.md){ .md-button }
[🤖 ROS Setup](ros-setup.md){ .md-button }
[📚 Back to Resources](resources.md){ .md-button }

</div>

---

*Last updated: Fall 2025 • [Back to Resources](resources.md)*
