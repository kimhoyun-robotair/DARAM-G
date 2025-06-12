# DARAMG-SIM 🌕🤖
---
A simulation of DARAM-G based on Gazebo Harmonic.
This package includes:

- 🛠️ Modeling files for the DARAM-G rover

- 🚀 Launch files to run RTAB-Map

- 🧭 Launch files to run AMCL and Nav2

**Refer to this tutorial to first download the files from the offline model repository, then either export the model path to ```GZ_SIM_RESOURCE_PATH``` or add the export command to your ```.bashrc```.**

- https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics?tab=readme-ov-file

**If the model or world file still cannot be opened and shows errors, please open the SDF file and update any hardcoded paths to models, meshes, or other resources so that they point to the correct locations on your system.**

**Also, don’t forget that you must edit the parameters and map files referenced inside the launch files. All launch files currently use the parameters and map files from kimhoyun-robotair’s setup.**

**Main 1. Running Autonomous Exploration with RTAB-MAP, Nav2, m-Explore 🗺️**
```bash
# Terminal 1
cd /path/to/DARAM-G
source install/setup.bash
ros2 launch daramg_sim autonomous_exploration.launch.py

# Terminal 2
cd /path/to/m-explore-ros2
source install/setup.bash
ros2 launch explore_lite explore.launch.py
```

**Main 2. Running Autonomous Exploration with slam-toolbox, Nav2, m-Explore 🗺️**
```bash
# Terminal 1
cd /path/to/DARAM-G
source install/setup.bash
ros2 launch daramg_sim slamtoolbox_nav2.launch.py

# Terminal 2
cd /path/to/m-explore-ros2
source install/setup.bash
ros2 launch explore_lite explore.launch.py
```

**2. Running AMCL and Nav2 🧭🤖**
```bash
# Terminal 1
source install/setup.bash
ros2 launch daramg_sim amcl.launch.py

# Terminal 2
source install/setup.bash
ros2 launch daramg_sim navigation.launch.py
```

**3. Using RTAB-Map and Nav2 without AMCL 💡**
If you want to use RTAB-Map with Nav2 without AMCL, follow these steps:

- Open rtab_map.launch.py.
- Set the localization launch argument to True.
- Do not launch AMCL.
- Only launch the following two files:
```bash
rtab_map.launch.py
navigation.launch.py
```

**4. If you want to check whether your robot is functioning properly**
If you want to verify that your robot is working correctly in both Gazebo and RViz, please run the following commands:

```bash
source install/setup.bash
ros2 launch daramg_sim spawn_robot.launch.py
```

**5. If you want to run Cartographer**
If you want to perform SLAM using Cartographer, use the following commands:

```bash
source install/setup.bash
ros2 launch daramg_sim cartographer.launch.py
```
