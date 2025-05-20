# DARAMG-SIM 🌕🤖
---
A simulation of DARAM-G based on Gazebo Harmonic.
This package includes:

- 🛠️ Modeling files for the DARAM-G rover

- 🚀 Launch files to run RTAB-Map

- 🧭 Launch files to run AMCL and Nav2

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

