# Fusion System Development

The ROS project for the CPSL fusion team.

We use ROS2 Humble for this project.

## Quickstart

Here's the procedure to run a simple test node to see how things work.

```
mkdir -p /your/path/to/project/src  # note the src at the end
cd /your/path/to/project/src
git clone https://github.com/cpsl-research/fusion-system.git
cd ..
colcon build --packages-select fusion_system_bringup fusion_system_display
source install/setup.zsh  # if your shell is zsh, install/setup.bash if bash
ros2 launch fusion_system_bringup test_launch.xml
```
