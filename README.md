
# HybridVLA Real World Experiment

This code is to validate the openvla pipeline in the real world experiment, which built based on [HybridVLA](https://github.com/PKU-HMI-Lab/Hybrid-VLA).
We have the VLA runner site and Franka controller site, VLA site recevies the image and promt into VLA model and send the predicted action to the Franka site.


# Installation
Our installation includes two parts.


- **VLA Runner Site**

Create an environment, clone the repo and install the required packages.

```bash
# Create environment
conda create --name hybridvla python=3.10

# Create environment
git clone https://github.com/hca-lab/HybridVLA-RealWorld.git
cd HybridVLA-RealWorld
pip install -e .
```


- **Franka Controller Site**: 

Install franky

```bash
# Install the franky
pip install franky-control
```
Please move the folder 'FrankyControl' to the Franka controller site.


# Checkpoints Download
Please download the HybridVLA release checkpoints:
- [Robotic Large-Scale Pretrained Checkpoint](https://pan.baidu.com/s/134S9y8UwoNlyw3yUKozbRw?pwd=1spu)
- [Simulation-Finetuned Checkpoint](https://pan.baidu.com/s/1f5zpPKoAJDRIHFIH602Bqg?pwd=3ca1)


# Real-World Experiment

Please run the Franka site and then run the VLA site.

- **Run Franka Site**


```bash
# run test_vla.py file
python test_vla.py
```

```bash
# To quit the control, press 'CTR'+'C'.
```

Please change the IP address based on your setting and adjust the scale parameter on action to avoid large movement.

- **Run VLA Site**

```bash
# run run_vla.py file
python scripts/run_vla.py 
```
Please change the IP address based on your setting and the checkpoints path. We use two RealSense cameras for wrist and front views. Please ensure that the image inputs to the VLA model are correctly assigned.

```bash
# To quit the control, press 'q'.
```
