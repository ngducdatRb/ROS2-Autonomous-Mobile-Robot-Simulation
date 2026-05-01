import math

import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

from matplotlib.axes import Axes

from typing import Optional, NamedTuple
from dataclasses import dataclass, fields

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ──────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────
TOPIC_GROUND_TRUTH    = "/model/robot/pose"
TOPIC_ODOM_DIFF_DRIVE = "/diff_drive/odom"
TOPIC_ODOM_PUBLISHER  = "/odom_publisher/odom"
TOPIC_ODOM_FILTERED   = "/odometry/filtered"

all_topics = {
    TOPIC_GROUND_TRUTH, 
    TOPIC_ODOM_DIFF_DRIVE, 
    TOPIC_ODOM_PUBLISHER, 
    TOPIC_ODOM_FILTERED
}

ROBOT_FRAME = "robot"

# ──────────────────────────────────────────────────────────────────
# PATH
# ──────────────────────────────────────────────────────────────────
PATH_BAG  = "/home/d4z/ros2/robot_sim_ws/src/robot_bag/odom_drift_2"
PATH_IMG  = "/home/d4z/ros2/robot_sim_ws/src/robot_metrics/images/plot_error_2.png"

PATH_FONT_SPACEMONO  = "/home/d4z/.local/share/fonts/SpaceMono-Regular.ttf"
PATH_FONT_ROBOTOSLAB = "/home/d4z/.local/share/fonts/RobotoSlab-VariableFont_wght.ttf"

# ──────────────────────────────────────────────────────────────────
# DATACLASS
# ──────────────────────────────────────────────────────────────────
@dataclass
class ParsedPose:
    time: float
    x   : float
    y   : float
    yaw : float

@dataclass
class Trajectory:
    time: np.ndarray
    x   : np.ndarray
    y   : np.ndarray
    yaw : np.ndarray

    @classmethod
    def empty(cls) -> "Trajectory":
        return cls(time=[], x=[], y=[], yaw=[])
    
    def to_numpy(self) -> "Trajectory":
        new_data = {f.name: np.array(getattr(self, f.name))
            for f in fields(self)            
        }
        return Trajectory(**new_data)

    def sort_by_time(self) -> "Trajectory":
        idx = np.argsort(self.time)
        new_data = {f.name: getattr(self, f.name)[idx]
            for f in fields(self)
        }
        return Trajectory(**new_data)

    def append(self, pose: ParsedPose) -> None:
        self.time.append(pose.time)
        self.x.append(pose.x)
        self.y.append(pose.y)
        self.yaw.append(pose.yaw)
        
@dataclass
class RosbagData:
    ground_truth   : Trajectory
    odom_diff_drive: Trajectory
    odom_publisher : Trajectory
    odom_filtered  : Trajectory

@dataclass
class ErrorResult:
    err_x   : np.ndarray
    err_y   : np.ndarray
    err_dist: np.ndarray
    err_yaw : np.ndarray
    metrics : dict

@dataclass
class ErrorBundle:
    odom_diff_drive: ErrorResult
    odom_publisher : ErrorResult
    odom_filtered  : ErrorResult

# ──────────────────────────────────────────────────────────────────
# NAMED TUPLE
# ──────────────────────────────────────────────────────────────────
class BagReader(NamedTuple):
    reader      : rosbag2_py.SequentialReader
    msg_type_map: dict[str, str]

# ──────────────────────────────────────────────────────────────────
# UTILITIES
# ──────────────────────────────────────────────────────────────────
def quaternion_to_yaw(quat) -> float:
    sin_yaw = 2 * (quat.w * quat.z + quat.x * quat.y)
    cos_yaw = 1 - 2 * (quat.y ** 2 + quat.z ** 2)
    return math.atan2(sin_yaw, cos_yaw)


# ──────────────────────────────────────────────────────────────────
# READ ROSBAG
# ──────────────────────────────────────────────────────────────────
def _parse_odom_msg(msg: Odometry) -> ParsedPose:
    time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    x    = msg.pose.pose.position.x
    y    = msg.pose.pose.position.y
    yaw  = quaternion_to_yaw(msg.pose.pose.orientation)
    return ParsedPose(time, x, y, yaw)

def _parse_pose_msg(msg: PoseStamped) -> ParsedPose:        
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x    = msg.pose.position.x
        y    = msg.pose.position.y
        yaw  = quaternion_to_yaw(msg.pose.orientation)
        return ParsedPose(time, x, y, yaw)

def _open_reader(path_bag: str) -> BagReader:
    option_storage   = rosbag2_py.StorageOptions(uri=path_bag, storage_id="sqlite3")    # sqlite3 = .db3
    option_converter = rosbag2_py.ConverterOptions("cdr", "cdr")                        # input: cdr → output: cdr

    reader = rosbag2_py.SequentialReader()
    reader.open(option_storage, option_converter)

    topic_types  = reader.get_all_topics_and_types()
    msg_type_map = {t.name: t.type for t in topic_types}

    return BagReader(reader, msg_type_map)

def _collect_trajectories(bag_reader: BagReader) -> RosbagData:
    ground_truth    = Trajectory.empty()
    odom_diff_drive = Trajectory.empty()
    odom_publisher  = Trajectory.empty()
    odom_filtered   = Trajectory.empty()

    while bag_reader.reader.has_next():
        topic_name, raw_data, bag_timestamp = bag_reader.reader.read_next()
        if topic_name not in all_topics:
            continue

        msg = deserialize_message(raw_data, get_message(bag_reader.msg_type_map[topic_name]))

        if topic_name == TOPIC_GROUND_TRUTH:
            pose = _parse_pose_msg(msg)
            ground_truth.append(pose)

        else:
            pose = _parse_odom_msg(msg)
            if topic_name == TOPIC_ODOM_DIFF_DRIVE:
                odom_diff_drive.append(pose)
            elif topic_name == TOPIC_ODOM_PUBLISHER:
                odom_publisher.append(pose)
            elif topic_name == TOPIC_ODOM_FILTERED:
                odom_filtered.append(pose)
    
    ground_truth, odom_diff_drive, odom_publisher, odom_filtered = [
        traj.to_numpy().sort_by_time()
        for traj in (ground_truth, odom_diff_drive, odom_publisher, odom_filtered)
    ]

    return RosbagData(ground_truth, odom_diff_drive, odom_publisher, odom_filtered)

def _validate_trajectories(trajectories: RosbagData) -> None:
    checks = {
        "Ground Truth"   : trajectories.ground_truth,
        "Odom Diff Drive": trajectories.odom_diff_drive,
        "Odom Publisher" : trajectories.odom_publisher,
        "Odom Filtered"  : trajectories.odom_filtered
    }

    for name, traj in checks.items():
        if len(traj.time) == 0:
            raise RuntimeError(f"No data from: {name}")

def read_rosbag(path_bag: str) -> RosbagData:
    bag_reader   = _open_reader(path_bag)
    trajectories = _collect_trajectories(bag_reader)
    valid_rosbag = _validate_trajectories(trajectories)

    return RosbagData(
        trajectories.ground_truth, 
        trajectories.odom_diff_drive, 
        trajectories.odom_publisher, 
        trajectories.odom_filtered
    )

# ──────────────────────────────────────────────────────────────────
# TIME ALIGNMENT & INTERPOLATION
# ──────────────────────────────────────────────────────────────────
def align_trajectories(gt: Trajectory, src: Trajectory) -> tuple[Trajectory, Trajectory]:
    rel_gt  = gt.time - gt.time[0]        # rel_gt = relative time ground truth (bag timestamp)
    rel_src = src.time - src.time[0]      # rel_src = relative time source (sim time)

    duration    = min(rel_gt[-1], rel_src[-1])
    common_time = rel_gt[rel_gt <= duration]

    if len(common_time) == 0:
        raise RuntimeError("No common time between GT and Src")
    
    unwrap_yaw_gt  = np.unwrap(gt.yaw)
    unwrap_yaw_src = np.unwrap(src.yaw)
        
    interp_yaw_gt  = np.interp(common_time, rel_gt, unwrap_yaw_gt)
    interp_yaw_src = np.interp(common_time, rel_src, unwrap_yaw_src)

    interp_yaw_src += interp_yaw_gt[0] - interp_yaw_src[0]

    interp_gt = Trajectory(
        time = common_time,
        x    = np.interp(common_time, rel_gt, gt.x),
        y    = np.interp(common_time, rel_gt, gt.y),
        yaw  = interp_yaw_gt,
    )

    interp_src = Trajectory(
        time = common_time,
        x    = np.interp(common_time, rel_src,  src.x),
        y    = np.interp(common_time, rel_src, src.y),
        yaw  = interp_yaw_src,
    )
    return interp_gt, interp_src

# ──────────────────────────────────────────────────────────────────
# ERROR COMPUTATION
# ──────────────────────────────────────────────────────────────────
def compute_errors(gt: Trajectory, src: Trajectory) -> ErrorResult:
    err_x    = src.x - gt.x
    err_y    = src.y - gt.y
    err_dist = np.sqrt(err_x ** 2 + err_y ** 2)
    err_yaw  = np.degrees(src.yaw - gt.yaw)

    return ErrorResult(
        err_x    = err_x,
        err_y    = err_y,
        err_dist = err_dist,
        err_yaw  = err_yaw,
        metrics  = {
            "max_error_x"   : np.max(np.abs(err_x)),
            "max_error_y"   : np.max(np.abs(err_y)),
            "max_err_dist"  : np.percentile(err_dist, 99),
            "max_err_yaw"   : np.max(np.abs(err_yaw)),
            "final_drift"   : err_dist[-1],
            "rmse_position" : np.sqrt(np.mean(err_dist ** 2)),
            "rmse_yaw"      : np.sqrt(np.mean(err_yaw  ** 2))
        }
    )

# ──────────────────────────────────────────────────────────────────
# VISUALIZATION
# ──────────────────────────────────────────────────────────────────
def _plot_trajectories(ax: Axes, trajs: RosbagData) -> None:
    ax.plot(trajs.ground_truth.x,    trajs.ground_truth.y,    'r-',  linewidth=2,   label="Ground Truth")
    ax.plot(trajs.odom_diff_drive.x, trajs.odom_diff_drive.y, 'g--', linewidth=1.2, label="Odom Diff Drive")
    ax.plot(trajs.odom_publisher.x,  trajs.odom_publisher.y,  'k:',  linewidth=3,   label="Odom Publisher")
    ax.plot(trajs.odom_filtered.x,   trajs.odom_filtered.y,   'b-',  linewidth=1.5, label="Odom Filtered")
    ax.set_title("2D Trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize='9', framealpha=0.8)

def _plot_error_distance(ax: Axes, trajs: RosbagData, err: ErrorBundle) -> None:
    ax.plot(trajs.odom_diff_drive.time, err.odom_diff_drive.err_dist, 'g--', linewidth=1.2, label="Odom Diff Drive")
    ax.plot(trajs.odom_publisher.time,  err.odom_publisher.err_dist,  'k:',  linewidth=3,   label="Odom Publisher")
    ax.plot(trajs.odom_filtered.time,   err.odom_filtered.err_dist,   'b-',  linewidth=1.5, label="Odom Filtered")
    ax.fill_between(trajs.odom_diff_drive.time, 0, err.odom_diff_drive.err_dist, alpha=0.1, color='green')
    ax.fill_between(trajs.odom_filtered.time,   0, err.odom_filtered.err_dist,   alpha=0.1, color='blue')
    ax.set_title("Error Distance")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()

def _plot_error_yaw(ax: Axes, trajs: RosbagData, err: ErrorBundle) -> None:
    ax.plot(trajs.odom_diff_drive.time, err.odom_diff_drive.err_yaw, 'g--', linewidth=1.2, label="Odom Diff Drive")
    ax.plot(trajs.odom_publisher.time,  err.odom_publisher.err_yaw,  'k:',  linewidth=3,   label="Odom Publisher")
    ax.plot(trajs.odom_filtered.time,   err.odom_filtered.err_yaw,   'b-',  linewidth=1.5, label="Odom Filtered")
    ax.fill_between(trajs.odom_diff_drive.time, 0, err.odom_diff_drive.err_yaw, alpha=0.1, color='green')
    ax.fill_between(trajs.odom_filtered.time,   0, err.odom_filtered.err_yaw,   alpha=0.1, color='blue')
    ax.set_title("Error Yaw")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()

def _plot_metric_table(ax: Axes, err: ErrorBundle) -> None:
    ax.axis('off')

    metric_labels = {
        "max_err_dist"  : ("Max error dist", "m",  ".4f"),
        "final_drift"   : ("Final drift",    "m",  ".4f"),
        "rmse_position" : ("RMSE position",  "m",  ".4f"),
        "max_err_yaw"   : ("Max error yaw",  "°",  ".4f"),
        "rmse_yaw"      : ("RMSE yaw",       "°",  ".4f"),
    }

    rows = []
    for key, (label, unit, fmt) in metric_labels.items():
        rows.append([
            label,
            f"{err.odom_diff_drive.metrics[key]:{fmt}} {unit}",
            f"{err.odom_publisher.metrics[key]:{fmt}} {unit}",
            f"{err.odom_filtered.metrics[key]:{fmt}} {unit}",
        ])
    
    imp_ekf = (err.odom_diff_drive.metrics['rmse_position'] - err.odom_filtered.metrics['rmse_position']) / err.odom_diff_drive.metrics['rmse_position'] * 100
    rows.append(["RMSE improvement", "—", "—", f"{imp_ekf:.1f} %"])

    table = ax.table(
        cellText  = rows,
        colLabels = ["Metric", "Odom Diff Drive", "Odom Publisher", "Odom Filtered"],
        loc       = 'center',
        cellLoc   = 'center', 
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 2.7)
    ax.set_title("Summary Metrics", pad=6)

def plot_results(trajs: RosbagData, err: ErrorBundle) -> None:
    fm.fontManager.addfont(PATH_FONT_ROBOTOSLAB)
    font_prop = fm.FontProperties(fname=PATH_FONT_ROBOTOSLAB)
    plt.rcParams["font.family"] = font_prop.get_name()

    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    fig.suptitle("Error Analysis", fontsize=14, fontweight="bold", fontproperties=font_prop)
    
    _plot_trajectories(axes[0, 0], trajs)
    _plot_error_distance(axes[0, 1], trajs, err)
    _plot_error_yaw(axes[1, 0], trajs, err)
    _plot_metric_table(axes[1, 1], err)

    plt.tight_layout()
    plt.savefig(PATH_IMG, dpi=150)
    plt.show()
    print(f"Saved → {PATH_IMG}")


# ──────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────
def main():
    print("Reading Bag File ...")
    trajs = read_rosbag(PATH_BAG)
    print(
        f"Ground Truth: {len(trajs.ground_truth.time)} samples | "
        f"Odom Diff Drive: {len(trajs.odom_diff_drive.time)} samples | "
        f"Odom Publisher: {len(trajs.odom_publisher.time)} samples | "
        f"Odom Filtered: {len(trajs.odom_filtered.time)} samples | "
    )

    gt_for_odom_diff_drive, interp_odom_diff_drive = align_trajectories(trajs.ground_truth, trajs.odom_diff_drive)
    gt_for_odom_publisher,  interp_odom_publisher  = align_trajectories(trajs.ground_truth, trajs.odom_publisher)
    gt_for_odom_filtered,   interp_odom_filtered   = align_trajectories(trajs.ground_truth, trajs.odom_filtered)

    err_odom_diff_drive = compute_errors(gt_for_odom_diff_drive, interp_odom_diff_drive)
    err_odom_publisher  = compute_errors(gt_for_odom_publisher,  interp_odom_publisher)
    err_odom_filtered   = compute_errors(gt_for_odom_filtered,   interp_odom_filtered)

    trajs = RosbagData(gt_for_odom_diff_drive, interp_odom_diff_drive, interp_odom_publisher, interp_odom_filtered)
    error = ErrorBundle(err_odom_diff_drive, err_odom_publisher, err_odom_filtered)
    plot_results(trajs, error)

if __name__ == "__main__":
    main()
