# 🚢 USV Control Simulation with Docker, ROS2 Humble, and Gazebo 11

This guide provides step-by-step instructions for setting up a USV (Unmanned Surface Vehicle) control simulation using Docker, ROS2 Humble, and Gazebo 11. The integration leverages ROS2 for efficient communication and control within the Gazebo simulation environment.

<p align="center">
  <img src="https://github.com/Eminkorkut/usvDocker/blob/main/image/docker-logo.png" alt="Gazebo 11" width="800"/>
</p>


<h2>🛠️ Installation Steps:</h2>

<p1>Set up Docker's apt repository</p1>
```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

<p1>Install the Docker packages</p1>
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

