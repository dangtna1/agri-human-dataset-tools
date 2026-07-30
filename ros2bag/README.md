# ROS2 Bag Converter (Dataset → ROS 2 rosbag2)

This tool **checks a dataset folder for consistency** (sensor files, annotations, metadata)
and then **converts it into a ROS 2 rosbag2 directory** that can be played in ROS 2 Humble
and visualized in RViz2.

> **Note**
> A ROS 2 bag is a **directory**, not a single file.
> In this README, `<ROS2_BAG_DIR>` refers to the output directory passed to `--rosbag-out`.

---

## Expected dataset structure

```
<DATASET_BAG_DIR>/
├── sensor_data/
│   ├── cam_zed_rgb/        *.png
│   ├── cam_zed_depth/      *.npy
│   ├── cam_fish_front/     *.png
│   ├── cam_fish_left/      *.png
│   ├── cam_fish_right/     *.png
│   └── lidar/              *.pcd
├── annotations/
│   ├── cam_zed_rgb_ann.json
│   ├── cam_fish_front_ann.json
│   ├── cam_fish_left_ann.json
│   ├── cam_fish_right_ann.json
│   └── lidar_ann.json
├── metadata/
│   ├── *.jsonl
│   └── tf/
│       ├── map__to__odom.jsonl
│       └── odom__to__base_link.jsonl
└── sync.json   (optional)
```

---

## What the script does

### 1) Dataset checks (fail fast)

Before any conversion, the script validates:

- Sensor files
  - Counts files per modality
  - Extracts timestamps from filenames
- Annotations (`*_ann.json`)
  - Every `File` entry exists under `sensor_data/`
  - Annotation `Timestamp` matches the filename timestamp within a tolerance (default 5 ms)
- Metadata (`*.jsonl`)
  - Lines parse as valid JSON
  - Reports non-monotonic timestamps (does not fail by default)
- `sync.json`
  - Optional; missing file only produces a warning

If any annotation references are missing or timestamps do not match,
the script **stops immediately** and does **not** create a rosbag.

---

## ROS 2 bag output

### Sensor topics

| Dataset modality | ROS topic | Message type |
|---|---|---|
| cam_fish_front | /dataset/cam_fish_front/image | sensor_msgs/msg/Image |
| cam_fish_left | /dataset/cam_fish_left/image | sensor_msgs/msg/Image |
| cam_fish_right | /dataset/cam_fish_right/image | sensor_msgs/msg/Image |
| cam_zed_rgb | /dataset/cam_zed_rgb/image | sensor_msgs/msg/Image |
| cam_zed_depth | /dataset/cam_zed_depth/image | sensor_msgs/msg/Image |
| lidar | /dataset/lidar/points | sensor_msgs/msg/PointCloud2 |

When `--write-calibration` is used for the AGHRI ZED RGB/Livox workflow:

| Source | ROS topic | Message type | Frame ID |
|---|---|---|---|
| `sensor_data/cam_zed_rgb/*.png` | `/dataset/cam_zed_rgb/image` | `sensor_msgs/msg/Image` | `front_left_camera_optical_frame` |
| `calibration/intrinsics.json` | `/dataset/cam_zed_rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | `front_left_camera_optical_frame` |
| `sensor_data/cam_fish_front/*.png` | `/dataset/cam_fish_front/image` | `sensor_msgs/msg/Image` | `fish_front_camera_link_optical` |
| `calibration/intrinsics.json` | `/dataset/cam_fish_front/camera_info` | `sensor_msgs/msg/CameraInfo` | `fish_front_camera_link_optical` |
| `sensor_data/cam_fish_left/*.png` | `/dataset/cam_fish_left/image` | `sensor_msgs/msg/Image` | `fish_left_camera_link_optical` |
| `calibration/intrinsics.json` | `/dataset/cam_fish_left/camera_info` | `sensor_msgs/msg/CameraInfo` | `fish_left_camera_link_optical` |
| `sensor_data/cam_fish_right/*.png` | `/dataset/cam_fish_right/image` | `sensor_msgs/msg/Image` | `fish_right_camera_link_optical` |
| `calibration/intrinsics.json` | `/dataset/cam_fish_right/camera_info` | `sensor_msgs/msg/CameraInfo` | `fish_right_camera_link_optical` |
| `sensor_data/lidar/*.pcd` | `/dataset/lidar/points` | `sensor_msgs/msg/PointCloud2` | `front_lidar_link` |
| `--write-tf` | `/tf` | `tf2_msgs/msg/TFMessage` | dynamic transform, e.g. `map -> base_link` |
| `calibration/extrinsics.json` | `/tf_static` | `tf2_msgs/msg/TFMessage` | static transform tree |

The converter does not undistort or rectify images. It publishes the PNG files
as stored and translates the JSON calibration values into CameraInfo (`D`, `K`,
`R`, `P`) for the requested camera streams.

---

### Label topics (ground truth)

| Source | ROS topic | Message type |
|---|---|---|
| cam_*_ann.json | /dataset/labels/<camera> | vision_msgs/msg/Detection2DArray |
| lidar_ann.json | /dataset/labels/lidar | vision_msgs/msg/Detection3DArray |

For numeric annotation classes, each detection keeps the original zero-padded
person identity (for example `01`) in `detection.id`. The hypothesis
`class_id` is the semantic label `person`.

---

### Optional visualization

- /dataset/viz/lidar_boxes (visualization_msgs/msg/MarkerArray)

---

### TF handling

- Dynamic `/tf` (map → lidar) is still available through `--write-tf`.
- Calibration-aware AGHRI conversion writes static sensor transforms from
  `calibration/extrinsics.json` to `/tf_static`.
- Original image and LiDAR timestamps are preserved. The converter does not
  force camera and LiDAR messages onto matching timestamps.

---

## Requirements

- ROS 2 Humble
- Python 3
- vision_msgs
- numpy
- Pillow

---

## Usage

### Check only

```bash
python3 check_and_make_rosbag2.py --bag-dir <DATASET_BAG_DIR>
```

### Convert to rosbag2

```bash
rm -rf <ROS2_BAG_DIR>

python3 check_and_make_rosbag2.py \
  --bag-dir <DATASET_BAG_DIR> \
  --make-rosbag \
  --rosbag-out <ROS2_BAG_DIR>
```

### Convert with TF

```bash
rm -rf <ROS2_BAG_DIR>

python3 check_and_make_rosbag2.py \
  --bag-dir <DATASET_BAG_DIR> \
  --make-rosbag \
  --rosbag-out <ROS2_BAG_DIR> \
  --write-tf \
  --tf-parent map \
  --tf-child lidar \
  --tf-xyzrpy 0,0,0,0,0,0
```

### Convert with AGHRI ZED RGB/Livox calibration

```bash
rm -rf <ROS2_BAG_DIR>

python3 check_and_make_rosbag2.py \
  --bag-dir <DATASET_BAG_DIR> \
  --make-rosbag \
  --rosbag-out <ROS2_BAG_DIR> \
  --write-tf \
  --tf-parent map \
  --tf-child base_link \
  --tf-xyzrpy 0,0,0,0,0,0 \
  --write-calibration \
  --calibration-path /media/prabuddhi/Backup2/Updated\ Dataset_PW/calibration.zip \
  --intrinsics-member calibration/intrinsics.json \
  --extrinsics-member calibration/extrinsics.json \
  --camera-name cam_zed_rgb \
  --camera-info-topic /dataset/cam_zed_rgb/camera_info \
  --camera-info-cameras cam_zed_rgb,cam_fish_front,cam_fish_left,cam_fish_right
```

This mode writes one CameraInfo at each selected camera image timestamp, a
dynamic `/tf` stream at LiDAR timestamps, and one `/tf_static` message
containing the static transforms from the calibration JSON.

On ROS 2 Humble, play calibration-aware bags with a QoS override that makes
`/tf_static` transient-local, for example:

```yaml
/tf_static:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
```

### Play

```bash
ros2 bag play <ROS2_BAG_DIR> --clock 
```

Note: Make sure **--clock** is used

---

## RViz2

- Fixed Frame: `front_lidar_link` for calibration-aware AGHRI bags, or `lidar`
  for legacy non-calibrated bags.
- Add PointCloud2: /dataset/lidar/points
- Add MarkerArray: /dataset/viz/lidar_boxes
