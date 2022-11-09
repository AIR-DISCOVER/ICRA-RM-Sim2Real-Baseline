# ICRA Robomaster Sim2real Challenge Client Baseline

## Task

The task of this challenge (Phase 1) is to grasp the orbs (randomly located cubes) and place the orb at the correct location of the checking station. 

At launch time, three targets is spawned in the area. Less time used to correctly complete all three tasks leads to higher ranks in the leaderboard.

## Hardware - EP

The [RoboMaster EP](https://www.dji.com/cn/robomaster-ep) is an autonomous vehicle platform equipped with a 4 axis palletizing robot arm and an encircling gripper for flexible gripping action, inspired from DJI's annual RoboMaster robotics competition. It provides Mecanum wheel omnidirectional movement, fast wireless data link system including realtime video stream, and open sdk for further development and programming.

In order to match the theme of the course, we equip the RoboMaster EP with an external computing platform (NUC), as well as additional sensors including **onboard lidar** and **RGB-D cameras** for the purposes of better perception of the environment.

An external computing platform is also prepared to communicate with EPs through RoboMaster EP SDK. This platform is dedicated to run real applications in both simulated and real environments.

![group](assets/group.png)

The specification of the equipped NUC is listed as follow:

| Model      | CPU        | RAM     | SSD     |
|  :--------------------: | :----------------------: | :----------------------: | :----------------------: |
|  NUC11PAHI7 | i7-1165G7 (2.8GHz, 8 Cores) | 8GB | 256GB |

## Software Architecture - Sim

```mermaid
graph LR

A([ROS Master])

S([Server])

C([Client])

subgraph Net Bridge

S-..ROS topics..->C

C-..ROS topics..->S

C---A
A---S

end
```

### Core (ROS Master)

This part serves as the communication pivot in ROS systems.

### Server

In phase 1, the task is performed in the simulated scene. Therefore, we build a digital twin of the testing scene in reality with `habitat-sim` and `habitat-lab`. This digital twin contains a refined simulation of the test ground used in later phase, as well as the simulation of the EP itself.

The interfaces between server and client are defined by ROS topics and the detailed definitions can be found [here](./reference.md).

### Client

In this repo, we provide a baseline method for this task. A functional image can be obtained by [building the image](#build-an-updated-client-image) or pulling `docker.discover-lab.com:55555/rm-sim2real/client:v1.0.1`.

Note that, due to the hardware resource constraints on EP, launched client should be limited to use less than 6 CPUs and 8192MB memory. These constraints are imposed by `--cpus=5.6 -m 8192M` arguments in `launch.sh`. Do not remove them during developing, or online testing performance may be significantly lower than local testing.

The baseline image (and potentially the images you built) is based on the base image `docker.discover-lab.com:55555/rm-sim2real/client-base:v1.4.0`. Basic functional units (e.g. [keyborad control]()) is included in this base image. Please refer to [this repo](https://github.com/AIR-DISCOVER/ICRA-RM-Sim2real-Client) for further information on the base image.

## How-to

### Launch server and client

The launching script `launch.sh` will create the docker network, the communicating core, the server and client container respectively:

```shell
scripts/launch.sh
```

You can use change the images used by specifying the environment variables `SERVER_IMAGE` and `CLIENT_IMAGE`:

```shell
# Suppose your client image is client-test:latest
CLIENT_IMAGE=client-test:latest scripts/launch.sh
```

This is especially useful when you are launching with the client image you built locally. 

By default if the launching is successful, three `image_view` windows and one `cartographer` window would appear on the screen:

![demo-launch](assets/launch-vis.png)

![demo-carto](assets/launch-carto.png)

### Stop running containers

This script will stop all running containers and remove the network right away:

```shell
scripts/halt.sh
```

### Open the shell of the client

This script will open a shell inside the client:

```shell
scripts/shell.sh
```

Note that, by default ROS commands (e.g. `rostopic`, `roslaunch`, `rosrun`) are ready to use without additional setup (no need to source `setup.bash` file).

Also, any changes made in a running container **DOES NOT** persist. It is suggested to use the shell for debugging only and use `docker build` (see [build the container](#build-the-container)) for persistent changes.

### Execute command inside the client

`shell.sh` also accepts commands as arguments. For example:

```shell
scripts/shell.sh rostopic list -v
```

### Manually control EP with keyboard

Use `scripts/kbd_control.sh` to manually control EP:

```shell
scripts/kbd_control.sh
```

Control reference:

* `i`: Move forward
* `j`: Rotate counter clock-wise
* `l`: Rotate clock-wise
* `,`: Move backward
* `J`: Move horizontally to left
* `L`: Move horizontally to right
* `k`: Stop moving
* `1`: Lower robot arm
* `2`: Lift robot arm
* `3`: Close gripper
* `4`: Open gripper
* `Ctrl + C`: Stop listening keyboard inputs

### Display the timer of placing cubes

`judging.sh` invokes `rostopic echo` which prints the `/judgement/markers_time` on the screen:

```shell
scripts/judging.sh
```

### Use Visual Studio Code to debug

Install the extension for remote developing of the docker container (ms-vscode-remote.remote-containers):

![demo-vscode-extension](assets/container-vscode-extension.png)

Click the remote development button in the left-bottom corner and then select "Attach to Running Container":

![demo-vscode-dev-container](assets/container-extension-select.png)

Then choose the running client container (i.e. `/client`) and then open the folder `/opt/ep_ws/src`. Now you should be able to make changes to the files inside the docker directly.

Note that any changes made in this way **DOES NOT** persist. Once the container is stopped and removed, changes will be lost. It is suggested to use the shell for debugging only and use `docker build` (see [build the container](#build-the-container)) for persistent changes.

### Build an updated client image

A `Dockerfile` is included in this repository. To build a newer client image, execute:

```shell
# Build with name `client-custom` and tag `latest`
docker build . -t client-custom:latest
```

By default, the codes for controling are located in `src/`. When building a client image, all codes under `src/` will be copied to `/opt/ep_ws/src/rmus_solution` in the image. 

Inside the image, `/opt/ep_ws` is the ROS workspace directory. Later in the dockerfile, `catkin_make` is run in this directory.


The entry for the client is fixed to `start.sh` in the root of repo. When the image is building, this script is copied to `/opt/start.sh` and serves as the main entry for the client. The content of this file can be changed arbitrarily to suit the requirements, but the name of the script should not be changed.

If additional apt and pip requirements are needed in the newer image, please modify `Dockerfile` to obtain these packages at compile time.

Change these lines to add more `apt` dependencies:

```Dockerfile
# Install extra dependencies with apt
RUN apt-get update && \
     apt-get install -y --no-install-recommends \
     ros-noetic-depthimage-to-laserscan ros-noetic-map-server python3-tf-conversions ros-noetic-global-planner && \
     rm -rf /var/lib/apt/lists/* && apt-get clean
```

Change these lines to add more `pip` dependencies:

```Dockerfile
# Install extra dependencies with pip
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple scipy
```

### Submit the image

Submitting images requires registering for the ICRA Robomaster Sim2real challenges. Please [sign up]() before proceeding.

If you wish to submit the image for online testing, please first tag your built image as:

```shell
# Suppose your image has the name of `client-custom` and tag `latest`
docker tag client-custom:latest docker.discover-lab.com:55555/[username]/client:[tag]
```

Replace `[username]` with your username during registration and replace `[tag]` with any tag you like. Note that, images whose name are not `client` will be ignored by the online testing system.

After that, push your image to our registry:

```shell
docker push docker.discover-lab.com:55555/[username]/client:[tag]
```

You may find the testing result of your image [here](http://103.242.175.254:11011).

## Reference

See [reference.md](./reference.md).