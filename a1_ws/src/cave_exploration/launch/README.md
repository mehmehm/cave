Autonomous exploration phase: A\* and A1 path following

This phase consumes the goal selected by frontier_detector.py, plans a safe
path through /projected_map, and follows the path using CHAMP's /cmd_vel
interface.

Data flow

frontier_detector publishes /cave/exploration/best_frontier.

astar_planner inflates occupied and unknown cells by the configured A1
radius, runs 8-connected A\*, and publishes /cave/exploration/path.

path_follower tracks the path conservatively and publishes /cmd_vel.

When the goal is reached, the frontier detector selects the next frontier
or recovery viewpoint and the cycle repeats.

Unknown cells are never treated as traversable. Diagonal corner cutting is
disabled. An empty or stale path makes the follower publish a zero velocity.

Install

Copy the scripts and launch file into the existing package:

cp astar_planner.py ~/a1_ws/src/cave_exploration/scripts/
cp path_follower.py ~/a1_ws/src/cave_exploration/scripts/
cp autonomous_exploration.launch ~/a1_ws/src/cave_exploration/launch/

dos2unix ~/a1_ws/src/cave_exploration/scripts/astar_planner.py
dos2unix ~/a1_ws/src/cave_exploration/scripts/path_follower.py

chmod +x ~/a1_ws/src/cave_exploration/scripts/astar_planner.py
chmod +x ~/a1_ws/src/cave_exploration/scripts/path_follower.py

python3 -m py_compile \
 ~/a1_ws/src/cave_exploration/scripts/astar_planner.py \
 ~/a1_ws/src/cave_exploration/scripts/path_follower.py

The package needs these runtime dependencies in package.xml:

<exec_depend>rospy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>tf</exec_depend>

For installation through catkin, add both scripts to the existing
catkin_install_python block in CMakeLists.txt:

catkin_install_python(PROGRAMS
scripts/frontier_detector.py
scripts/astar_planner.py
scripts/path_follower.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

Then build and source:

cd ~/a1_ws
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/a1_ws/devel/setup.bash --extend

Safe first run

Before launching, stop keyboard teleoperation and any other node that publishes
to /cmd_vel. Confirm the existing controller subscribes:

rostopic info /cmd_vel

Keep the cave, A1, timestamp repair, LIO-SAM, OctoMap, multimodal fusion,
voxel map, and frontier detector terminals running. Start this phase in a new
terminal:

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/a1_ws/devel/setup.bash --extend
roslaunch cave_exploration autonomous_exploration.launch

Keep Gazebo's emergency pause button available during the first test.

Verify before allowing sustained motion

rostopic echo -n 1 /cave/exploration/planner_status
rostopic echo -n 1 /cave/exploration/path
rostopic hz /cmd_vel
rostopic echo /cave/exploration/follower_status

Expected planner status:

PLANNED poses=... length=...m expansions=...

Expected follower states are ROTATING, FOLLOWING, and GOAL_REACHED.

In RViz, add a Path display with topic /cave/exploration/path and keep the
fixed frame as lio_odom.

Initial tuning

Keep maximum_linear_speed at 0.12 m/s for the first run.

Increase robot_radius if the A1 passes too close to cave walls.

Decrease it only if every valid route is rejected after checking the map.

If a velocity smoother is active, set cmd_vel_topic in the launch file to
that smoother's input topic instead of its output topic.

Never run keyboard teleoperation and autonomous following simultaneously.
