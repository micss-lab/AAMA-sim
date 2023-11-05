#!/bin/bash

# Docker installation script for Ubuntu 22.04

# Update the apt package index and install packages to allow apt to use a repository over HTTPS:
sudo apt update
sudo apt install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up the stable repository:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add your user to the docker group to execute docker commands without sudo
sudo usermod -aG docker $USER

# Enable and start Docker service
sudo systemctl enable docker
sudo systemctl start docker

# Check Docker version to confirm installation
docker --version

echo "Docker installation script finished."
