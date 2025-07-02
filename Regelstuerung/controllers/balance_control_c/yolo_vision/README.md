# YOLO Vision project

Python environment setup on Linux

```shell
conda create -n yolo_vision python=3.11 -y
conda activate yolo_vision
pip install -r requirements.txt
```

NVIDIA Setup 
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
CUDA Version Repository
https://developer.nvidia.com/cuda-toolkit-archive
```shell
sudo su
lspci | grep -i nvidia
gcc --version
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.3.1/local_installers/cuda-repo-ubuntu2004-12-3-local_12.3.1-545.23.08-1_amd64.deb
dpkg -i cuda-repo-ubuntu2004-12-3-local_12.3.1-545.23.08-1_amd64.deb
cp /var/cuda-repo-ubuntu2004-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/
apt-get update
apt-get -y install cuda-toolkit-12-3
reboot 
```

After Reboot 
```shell
export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
echo 'export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc
cd ..
git clone https://github.com/NVIDIA/cuda-samples.git
cd cuda-samples-master/Samples/6_Performance/transpose
make
./transpose
nvcc --version
```

Installieren von SMBus für RaspberryPi und Jetson Nano
https://raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2
```shell
sudo apt-get install python-smbus i2c-tools
```



Install labelme
```shell
sudo apt-get install labelme
labelme
```

Create images for annotation with ffmpeg. Sample line. Change params.
```shell
ffmpeg -i myvideo.avi -vf fps=1/60 img%03d.jpg
```

# Research 
- Masks from segmentation, find contours, find crucial points to draw 


Python environment setup on Windows

```commandline
python -m venv env\yolo_cpu
env\yolo_cpu\Scripts\activate
```
