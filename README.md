# 🚢 USV Control Simulation with Docker, ROS2 Humble, and Gazebo 11

This guide provides step-by-step instructions for setting up a USV (Unmanned Surface Vehicle) control simulation using Docker, ROS2 Humble, and Gazebo 11. The integration leverages ROS2 for efficient communication and control within the Gazebo simulation environment.

The simulation is based on the original code from the following repository:
🔗 [Original Code Repository](https://github.com/Eminkorkut/rosHumbleGazebo11USV)




<p align="center">
  <img src="https://github.com/Eminkorkut/usvDocker/blob/main/image/docker-logo.png" alt="Gazebo 11" width="800"/>
</p>


<h2>🛠️ Installation Steps:</h2>

<p1>Download if you don't have git</p1>
```bash
sudo apt install git
```

<p1>Clone the repo</p1>
```bash
git clone https://github.com/Eminkorkut/usvDocker.git
```

<p1>Get inside the repo</p1>
```bash
cd usvDocker
```

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

<h2>🔗 Useful Resources</h2>
<p>For further details on the technologies used in this project, refer to the official documentation:</p>
<ul>
  <li><a href="https://docs.docker.com/" target="_blank">Docker Documentation</a></li>
  <li><a href="https://docs.ros.org/en/humble/" target="_blank">ROS2 Humble Documentation</a></li>
  <li><a href="https://classic.gazebosim.org/tutorials?tut=install_ubuntu" target="_blank">Gazebo 11 Documentation</a></li>
  <li><a href="https://github.com/ultralytics/ultralytics" target="_blank">YOLOv11 GitHub Repository</a></li>
</ul>

<h2>🛠️ Stages of Use</h2>

<p1>Switch to work directory</p1>
```bash
cd usvDocker
```

<p1>Docker Build</p1>
```bash
# docker build -t <YOUR-IMAGES-NAME> .
docker build -t usvDocker .
```

<p1>Grant X11 access for Docker GUI applications</p1>
```bash
xhost +local:
```

<p1>Docker images run</p1>
```bash
docker run --rm --net=host --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="GAZEBO_AUDIO_DEVICE=" \
    --env="SDL_AUDIODRIVER=dummy" \
    --device /dev/snd \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -it usvDocker bash
```



<h2>🤝 Contribute to the Project</h2>
<p>We welcome contributions to improve this project! If you’d like to contribute, follow these steps:</p>
<ol>
  <li>Fork the repository: <a href="https://github.com/Eminkorkut/usvDocker/fork" target="_blank">Fork on GitHub</a></li>
  <li>Clone your forked repository:</li>
  <pre><code>git clone https://github.com/YOUR-NAME/usvDocker.git</code></pre>
  <li>Create a new branch:</li>
  <pre><code>git checkout -b feature-branch-name</code></pre>
  <li>Make your changes and commit them:</li>
  <pre><code>git add .
git commit -m "Describe your changes"
git push origin feature-branch-name</code></pre>
  <li>Open a Pull Request: <a href="https://github.com/Eminkorkut/usvDocker/pulls" target="_blank">Create a PR</a></li>
</ol>
<p>Thank you for your contributions! 🚀</p>


  
## ✍️ Authors

- [@Eminkorkut](https://github.com/Eminkorkut)
