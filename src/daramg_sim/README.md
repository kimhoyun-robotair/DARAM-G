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

**1. Running RTAB-Map 🗺️**
```bash
# Terminal 1
source install/setup.bash
ros2 launch daramg_sim spawn_robot.launch.py

# Terminal 2
source install/setup.bash
ros2 launch daramg_sim rtab_map.launch.py
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

