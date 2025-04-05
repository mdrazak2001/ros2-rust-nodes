# ROS2 Rust Nodes - Custom ROS 2 Workspace with Rust

🚀 This repository is a **ROS 2 workspace** (`ros2_ws`) that contains a set of ROS 2 nodes written in Rust that demonstrate publisher/subscriber communication using the [ros2_rust](https://github.com/ros2-rust/ros2_rust) client library.

## 1. Prerequisites

- **ROS 2 (ex Humble Hawksbill):** Follow the [ROS 2 Installation guide for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- **Rust Toolchain:** Install [Rust](https://www.rust-lang.org/tools/install).
- colcon (for building the workspace)


## 2. Setting Up Your Environment

### Step 1: Install and Build `ros2_rust`

Follow the instructions in [ros2-rust/ros2_rust](https://github.com/ros2-rust/ros2_rust) to install Rust bindings for ROS 2.
Once cloned and set up, build it inside your workspace (e.g., ~/workspace):

```bash
cd ~/workspace
colcon build
source install/setup.bash
```

### Step 2: Build This Workspace

This repository is already structured as a ROS 2 workspace. After cloning it, ensure it sources the `ros2_rust` build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash # Source the previously installed ros2 humble setup files
source ~/workspace/install/setup.bash  # Source ros2_rust
colcon build
source install/local_setup.bash
```
``Note: The source install/local_setup.bash ensures both ros2_rust and this package are properly linked.``

---

## 3. Running the Rust Nodes

The nodes now use ROS 2 parameters instead of command-line arguments. Parameters can be set using --ros-args --param when running the nodes.

#### Publisher Node Parameters
```bash
topic: <string>             # Name of the ROS2 topic to publish to  
publish_rate_ms: <integer>  # Publication rate in milliseconds  
prefix: <string>            # Text appearing before the counter  
suffix: <string>            # Text appearing after the counter  
count_start: <integer>      # Initial counter value  
```

#### Subscriber Node Parameters
```bash
topic: <string>        # Name of the ROS2 topic to subscribe to  
output_file: <string>  # Path to the file where received messages will be logged  

```


## Manual Run
You can run the nodes manually using the `ros2 run` command. For example:

**Publisher:**
```bash
ros2 run rust_nodes simple_publisher --ros-args \
  --param topic:=my_topic \
  --param publish_rate_ms:=1000 \
  --param prefix:="Hello" \
  --param suffix:="World" \
  --param count_start:=0
# Publishes: "Hello 0 World", "Hello 1 World", etc. every 1 second
```

**Subscriber:**
```bash
ros2 run rust_nodes simple_subscriber --ros-args \
  --param topic:=my_topic \
  --param output_file:=subscriber_output.txt
```

## Launch Files

For convenience, multi-level Python launch files are provided in `rust_nodes_bringup`:

```bash
ros2 launch rust_nodes_bringup main_launch.py publish_rate_ms:=5000 prefix:=Hi suffix:=There count_start:=10
```


## 4. Important Notes
⚠ Configuring `.cargo/config.toml`
This repository includes a `.cargo/config.toml` file, which must be modified for different environments:

- It contains local file paths that reference built dependencies in `ros2_rust`.

- Update the paths in .cargo/config.toml to match your setup after building ros2_rust.

- If the paths are incorrect, the build may fail due to missing dependencies.

Example:
If `ros2_rust` was built in `/home/user/my_ros2_rust_workspace`, update:

```toml
[patch.crates-io.rclrs]
path = "/home/user/my_ros2_rust_workspace/install/rclrs/share/rclrs/rust"
```
