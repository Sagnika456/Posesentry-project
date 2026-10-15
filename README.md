# PoseSentry

Real-time human pose estimation, posture classification and fall detection built on **ROS2** and **Gazebo**, with an accuracy evaluation node and support for a real webcam.

The pipeline detects 15 body keypoints from a camera image using a pretrained **OpenPose (MPI)** network through **OpenCV DNN**, draws the skeleton, classifies posture, flags possible falls, and measures its own accuracy against a known ground truth.

---

## Features

- **Pose estimation**: 15 keypoints and 14 limb pairs per frame (OpenPose MPI model, CPU inference)
- **Skeleton overlay**: annotated image published as a ROS2 topic
- **Posture classification and fall detection**: based on neck and hip keypoint angles
- **Accuracy evaluation**: pixel error against the simulated actor's ground-truth position, with a running average
- **Data analysis**: logged results turned into graphs with a Python script
- **Simulation input**: Gazebo world with a walking actor and a virtual camera
- **Real camera input**: webcam publisher that feeds the same pipeline

---

## How it works

```
Gazebo (walking actor + camera)  --or--  Real webcam
              |                                |
        ros_gz_bridge                    webcam_node.py
              \                                /
               ------------ /camera -----------
                              |
                    pose_estimation_node
        |             |              |                 |
 annotated_image   keypoints   posture_status   fall_detected
        |
 rqt_image_view            ground_truth_node --> accuracy_error
```

The pose node only subscribes to `/camera`, so it does not care whether the images come from Gazebo or a real webcam.

---

## ROS2 topics

| Topic | Type | Description |
|---|---|---|
| `/camera` | `sensor_msgs/Image` | Input image (from Gazebo bridge or webcam) |
| `/pose_estimation/annotated_image` | `sensor_msgs/Image` | Camera image with the skeleton drawn on it |
| `/pose_estimation/keypoints` | `geometry_msgs/PoseArray` | Detected keypoints, in image pixel coordinates |
| `/human/posture_status` | `std_msgs/String` | Posture label: `STANDING`, `BENDING`, `FALLEN` or `UNKNOWN` |
| `/human/fall_detected` | `std_msgs/Bool` | `true` when a fall is detected |
| `/pose_estimation/accuracy_error` | ROS2 message | Pixel error against ground truth |
| `/ground_truth/actor_position` | ROS2 message | Actor position from the known trajectory |

---

## Project structure

```
human_pose_estimation/
├── src/
│   ├── pose_estimation_node.cpp   # keypoint detection, posture, fall detection
│   └── ground_truth_node.cpp      # accuracy evaluation with running average
├── scripts/
│   └── webcam_node.py             # real webcam publisher to /camera
├── worlds/
│   └── human_pose_world.sdf       # Gazebo world (actor + camera)
├── analysis/                      # logs, graphs and analyze.py
├── models/                        # OpenPose MPI model files (see below)
├── CMakeLists.txt
└── package.xml
```

---

## Requirements

- Ubuntu with **ROS2** (developed on `lyrical`) and **Gazebo** (`gz sim`)
- Packages: `ros_gz_bridge`, `rqt_image_view`, `cv_bridge`, `image_transport`, OpenCV (with DNN module)
- Python 3 with OpenCV (for the webcam script)
- Python 3 with `pandas` and `matplotlib` (for the analysis script)

### Model files

Place the OpenPose MPI files in the package's `models/` folder:

- `pose_deploy.prototxt`
- `pose_weights.caffemodel`

The weights file is large (around 200 MB), so it may not be stored in this repository. Download it from the official OpenPose release and put it in `models/` before building.

---

## Build

```bash
mkdir -p ~/pose_estimation_ws/src
cd ~/pose_estimation_ws/src
git clone https://github.com/Sagnika456/Posesentry-project.git human_pose_estimation

cd ~/pose_estimation_ws
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash
```

---

## Run: simulation demo

Open one terminal per step. In **every** terminal first run:

```bash
source /opt/ros/lyrical/setup.bash
source ~/pose_estimation_ws/install/setup.bash
```

**1. Gazebo** (press the play button once it opens)
```bash
cd ~/pose_estimation_ws/src/human_pose_estimation/worlds
gz sim human_pose_world.sdf
```

**2. Camera bridge** (run as a single line)
```bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

**3. Pose estimation node**
```bash
ros2 run human_pose_estimation pose_estimation_node
```

**4. Accuracy node**
```bash
ros2 run human_pose_estimation ground_truth_node
```

**5. Viewer**
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/pose_estimation/annotated_image` in the dropdown. If the image looks sideways, set the rotation to 0°.

**Check posture and fall output**
```bash
ros2 topic echo /human/posture_status
ros2 topic echo /human/fall_detected
```

---

## Run: real webcam

Do not start Gazebo or the bridge. If you use a virtual machine, enable webcam passthrough first (VirtualBox: Devices → Webcams).

```bash
# terminal 1
ros2 run human_pose_estimation pose_estimation_node

# terminal 2
python3 scripts/webcam_node.py

# terminal 3
ros2 run rqt_image_view rqt_image_view
```

Stand far enough from the camera that your whole body is in frame.

---

## Data analysis

The `analysis/` folder holds the logged data, the script and the graphs:

- `gt_log.txt`: raw output of `ground_truth_node`
- `posture_log.txt`: logged `/human/posture_status` readings
- `accuracy_data.csv`: the extracted error values
- `analyze.py`: script that reads the logs and creates the graphs
- Five graphs: error per frame, running average, error histogram, error box plot, posture counts

To record new data and rebuild the graphs:

```bash
cd ~/analysis
ros2 run human_pose_estimation ground_truth_node 2>&1 | tee gt_log.txt
ros2 topic echo /human/posture_status > posture_log.txt
python3 analyze.py
```

**Results from 36 frames in simulation**

| Statistic | Error (px) |
|---|---|
| Mean | 3,885 |
| Median | 2,467 |
| Minimum | 1,312 |
| Maximum | 16,977 |

Posture readings (108 in total): 91 `STANDING`, 14 `UNKNOWN`, 2 `BENDING` and 1 `FALLEN`.

The error values are large because the ground-truth projection is not calibrated to the camera. A few large spikes pull the mean above the median.

---

## Known limitations

- **Speed**: inference runs on the CPU. In a virtual machine without a GPU, expect only a few frames per second, and a very low Gazebo real time factor.
- **Accuracy values**: the pixel error against ground truth is large because the ground-truth projection has not been calibrated to the camera.
- **Fall detection on a webcam**: it can give false alarms when only the upper body is visible, because hip keypoints are missing or unreliable. It was developed against the full-body simulated actor.
- **Keypoints are 2D**: they are published in pixel coordinates with no coordinate frame, so a proper RViz2 3D view is not implemented yet.

---

## Future work

- GPU inference for real-time speed
- Camera calibration to fix the ground-truth projection
- Conversion of keypoints to 3D and publishing them as RViz2 markers
- More robust fall detection that handles missing keypoints and uses motion over time

---

## Author

Sagnika, Computer Science Engineering
GitHub: [Sagnika456](https://github.com/Sagnika456)

---

## Credits

- [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) (MPI model) by the CMU Perceptual Computing Lab
- ROS2, Gazebo and OpenCV
