
# OpenAi ROS Integration

![CERA CRANIUM Architecture](https://media.springernature.com/lw685/springer-static/image/chp%3A10.1007%2F978-1-4614-0164-3_18/MediaObjects/193000_1_En_18_Fig3_HTML.gif)


## Overview

This repository integrates OpenAI Retro Gym with ROS, allowing you to control and visualize environments like Sonic the Hedgehog using ROS topics. It bridges the gap between Python 3.5 (required by OpenAI Gym) and Python 2.7 (used by ROS).

## Setup

1. Clone the repo in your ROS workspace:
   ```bash
   git clone https://github.com/dsapandora/s_cera
   ```

2. The issue with **OPENAI RETRO GYM** and **ROS** is that OPENAI works only with *Python 3.5* while ROS typically uses *Python 2.7*. More information can be found [here](https://stackoverflow.com/questions/49758578/installation-guide-for-ros-kinetic-with-python-3-5-on-ubuntu-16-04).

3. Inside the `s_cera` folder, create a virtual environment for Python 3.5:
   ```bash
   virtualenv -p python3 env
   source env/bin/activate
   pip3 install gym-retro
   pip3 install opencv-python
   pip3 install pygame
   pip3 install imutils
   pip3 install scipy
   pip3 install pyyaml
   pip3 install catkin_pkg
   pip3 install rospkg
   mkdir roms
   chmod +x retro_gym_server.py
   chmod +x viewer.py
   ```

4. This plugin is tested with Sonic the Hedgehog from Sega Genesis. You need to place the ROM in the `roms` folder. For more info about compatible ROMs, visit the [gym retro repository](https://github.com/openai/retro).

5. Import the ROM with the following command (execute in the `s_cera` folder):
   ```bash
   python3 -m retro.import roms
   ```

6. In your workspace, execute:
   ```bash
   catkin_make
   ```

## Usage

1. The server will run in Python 3 using the Python 3 environment. Before running the server, export the necessary paths:
   ```bash
   export PYTHONPATH="<S_CERA_FOLDER_PATH>/env/lib/python3.5/site-packages:$PYTHONPATH"
   rosrun s_cera retro_gym_server.py
   ```

2. To execute the client, open another console and run:
   ```bash
   rosrun s_cera viewer.py
   ```

This will open a Pygame window allowing you to control the agent using the keyboard. 

## Topics

The server publishes a compressed image topic (numpy matrix) named `world_observation/image_raw` and subscribes to `world_observation/cmd_vel` (a *Twist* message).

- **Linear.x**: Move the Sonic agent, 1 forward, -1 backward.
- **Linear.y**: Move the Sonic agent, 1 jump, -1 crouch.

The viewer publishes the `world_observation/cmd_vel` topic based on Pygame key events and subscribes to `world_observation/image_raw` to display real-time images from the server.

## Future Directions

- Implement OpenAI baselines and reinforcement learning algorithms.
- Explore advanced topics like policy distillation and meta-learning.

## References and Further Reading

- [OpenAI Retro Contest](https://blog.openai.com/retro-contest/)
- [OpenAI Baselines PPO](https://blog.openai.com/openai-baselines-ppo/)
- [Deep Reinforcement Learning](https://arxiv.org/abs/1710.02298)
- [Variational Autoencoder](https://jaan.io/what-is-variational-autoencoder-vae-tutorial/)
- [Conscious Robots](http://www.conscious-robots.com/consscale/)

## License

This project is licensed under the MIT License.

## Contributions

Contributions are welcome! Please fork this repository, make your changes, and submit a pull request.

## Contact

For any questions or support, please open an issue in this repository.

---
