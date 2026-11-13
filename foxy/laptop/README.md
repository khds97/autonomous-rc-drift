# Laptop

ROS 2 Foxy packages and tools intended to run on the Ubuntu laptop.

## Packages

| Package | Description |
|---|---|
| `driftmodel` | Laptop-side scripts, training code, and launch files |
| `driftmodel_msg` | Custom message and interface definitions |

## Requirements

- Ubuntu 20.04
- ROS 2 Foxy
- Git
- Colcon

## Get the laptop packages

The repository uses Git sparse checkout so the laptop and Raspberry Pi can use separate parts of the `foxy` branch.

Run the following commands on the laptop:

```bash
cd ~

git clone --filter=blob:none --sparse -b foxy \
  https://github.com/khds97/autonomous-rc-drift.git

cd ~/autonomous-rc-drift
git sparse-checkout set foxy/laptop
```

The laptop packages are available under:

```
foxy/laptop/
```

Check the selected directory with:

```bash
git sparse-checkout list
```

If you already cloned the full repository, you do not need to clone it again. From the repository directory, run:

```bash
git sparse-checkout init --cone
git sparse-checkout set foxy/laptop
```

Cone-mode sparse checkout also keeps files directly inside the repository root and the `foxy/` parent directory. The laptop-specific packages themselves remain under `foxy/laptop/`.

## Install dependencies

Make sure ROS 2 Foxy is installed, then run:

```bash
source /opt/ros/foxy/setup.bash
cd ~/autonomous-rc-drift/foxy/laptop

rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

If `rosdep` is not installed:

```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Build

Build the laptop packages:

```bash
cd ~/autonomous-rc-drift/foxy/laptop
source /opt/ros/foxy/setup.bash

colcon build --symlink-install
```

Source the workspace after building:

```bash
source install/setup.bash
```

For each new terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/autonomous-rc-drift/foxy/laptop/install/setup.bash
```

## Run

Run the required laptop-side nodes or launch files after building the workspace. Refer to the README files in the individual packages for their launch commands and configuration.

## Notes

- The laptop and Raspberry Pi use different sparse-checkout paths. On the laptop, use `git sparse-checkout set foxy/laptop`; on the Raspberry Pi, use `git sparse-checkout set foxy/RPi4`.
- If you need to change which part of the repository is checked out, run `git sparse-checkout set` again from the repository root.
- Build and test the packages on the laptop after installing their dependencies.
