import math
from typing import Dict, Tuple

import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ────────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ────────────────────────────────────────────────────────────────────────────────
GROUND_TRUTH_TOPIC = "/odom_publisher/odom"
ODOMETRY_TOPIC = "/diff_drive/odom"


# ────────────────────────────────────────────────────────────────────────────────
# DATA TYPES
# ────────────────────────────────────────────────────────────────────────────────
TrajectoryDict = Dict[str, np.ndarray]


# ────────────────────────────────────────────────────────────────────────────────
# UTILITIES
# ────────────────────────────────────────────────────────────────────────────────
def quaternion_to_yaw(quaternion) -> float:
    """Convert quaternion (geometry_msgs/Quaternion) to yaw (radians)."""
    sin_yaw = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2)
    return math.atan2(sin_yaw, cos_yaw)


def sort_trajectory_by_time(trajectory: TrajectoryDict) -> TrajectoryDict:
    """Ensure trajectory is sorted by timestamp."""
    sorted_indices = np.argsort(trajectory["time"])
    for key in trajectory:
        trajectory[key] = trajectory[key][sorted_indices]
    return trajectory


# ────────────────────────────────────────────────────────────────────────────────
# READ ROSBAG
# ────────────────────────────────────────────────────────────────────────────────
def read_rosbag(bag_path: str) -> Tuple[TrajectoryDict, TrajectoryDict]:
    """Read ground truth and odometry trajectories from rosbag."""

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    message_type_map = {topic.name: topic.type for topic in topic_types}

    ground_truth = {"time": [], "x": [], "y": [], "yaw": []}
    odometry = {"time": [], "x": [], "y": [], "yaw": []}

    while reader.has_next():
        topic_name, raw_data, _ = reader.read_next()

        if topic_name not in (GROUND_TRUTH_TOPIC, ODOMETRY_TOPIC):
            continue

        message = deserialize_message(
            raw_data, get_message(message_type_map[topic_name])
        )

        timestamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )

        position_x = message.pose.pose.position.x
        position_y = message.pose.pose.position.y
        yaw_angle = quaternion_to_yaw(message.pose.pose.orientation)

        target = (
            ground_truth if topic_name == GROUND_TRUTH_TOPIC else odometry
        )

        target["time"].append(timestamp)
        target["x"].append(position_x)
        target["y"].append(position_y)
        target["yaw"].append(yaw_angle)

    # Convert to NumPy arrays
    for trajectory in (ground_truth, odometry):
        for key in trajectory:
            trajectory[key] = np.array(trajectory[key])

    if len(ground_truth["time"]) == 0 or len(odometry["time"]) == 0:
        raise RuntimeError("Missing required topics in rosbag")

    return (
        sort_trajectory_by_time(ground_truth),
        sort_trajectory_by_time(odometry),
    )


# ────────────────────────────────────────────────────────────────────────────────
# TIME ALIGNMENT & INTERPOLATION
# ────────────────────────────────────────────────────────────────────────────────
def align_and_interpolate(
    ground_truth: TrajectoryDict,
    odometry: TrajectoryDict,
) -> Tuple[np.ndarray, TrajectoryDict, TrajectoryDict]:
    """Align trajectories in time and interpolate onto a common timeline."""

    start_time = max(ground_truth["time"][0], odometry["time"][0])
    end_time = min(ground_truth["time"][-1], odometry["time"][-1])

    common_time = ground_truth["time"][
        (ground_truth["time"] >= start_time)
        & (ground_truth["time"] <= end_time)
    ]

    odometry_interp = {
        "x": np.interp(common_time, odometry["time"], odometry["x"]),
        "y": np.interp(common_time, odometry["time"], odometry["y"]),
        "yaw": np.interp(
            common_time,
            odometry["time"],
            np.unwrap(odometry["yaw"]),
        ),
    }

    ground_truth_interp = {
        "x": np.interp(common_time, ground_truth["time"], ground_truth["x"]),
        "y": np.interp(common_time, ground_truth["time"], ground_truth["y"]),
        "yaw": np.interp(
            common_time,
            ground_truth["time"],
            np.unwrap(ground_truth["yaw"]),
        ),
    }

    relative_time = common_time - common_time[0]

    return relative_time, ground_truth_interp, odometry_interp


# ────────────────────────────────────────────────────────────────────────────────
# ERROR COMPUTATION
# ────────────────────────────────────────────────────────────────────────────────
def compute_errors(
    ground_truth: TrajectoryDict,
    odometry: TrajectoryDict,
):
    """Compute position and orientation errors."""

    error_x = odometry["x"] - ground_truth["x"]
    error_y = odometry["y"] - ground_truth["y"]

    distance_error = np.sqrt(error_x**2 + error_y**2)

    yaw_error = np.degrees(
        np.arctan2(
            np.sin(odometry["yaw"] - ground_truth["yaw"]),
            np.cos(odometry["yaw"] - ground_truth["yaw"]),
        )
    )

    metrics = {
        "max_error_x": np.max(np.abs(error_x)),
        "max_error_y": np.max(np.abs(error_y)),
        "max_distance_error": np.max(distance_error),
        "final_drift": distance_error[-1],
        "max_yaw_error_deg": np.max(np.abs(yaw_error)),
        "rmse_position": np.sqrt(np.mean(distance_error**2)),
    }

    return error_x, error_y, distance_error, yaw_error, metrics


# ────────────────────────────────────────────────────────────────────────────────
# VISUALIZATION
# ────────────────────────────────────────────────────────────────────────────────
def plot_results(
    time_axis: np.ndarray,
    ground_truth: TrajectoryDict,
    odometry: TrajectoryDict,
    error_x: np.ndarray,
    error_y: np.ndarray,
    distance_error: np.ndarray,
    yaw_error: np.ndarray,
    output_path: str,
):
    """Plot trajectory and error metrics."""

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle("Odometry Error Analysis", fontsize=14)

    # Trajectory plot
    ax = axes[0, 0]
    ax.plot(ground_truth["x"], ground_truth["y"], label="Ground Truth")
    ax.plot(odometry["x"], odometry["y"], "--", label="Odometry")
    ax.set_title("2D Trajectory")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()

    # Axis-wise error
    ax = axes[0, 1]
    ax.plot(time_axis, np.abs(error_x), label="X error")
    ax.plot(time_axis, np.abs(error_y), label="Y error")
    ax.set_title("Axis-wise Error")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.grid(True)
    ax.legend()

    # Distance error
    ax = axes[1, 0]
    ax.plot(time_axis, distance_error)
    ax.fill_between(time_axis, 0, distance_error, alpha=0.2, color='#CE93D8')
    ax.set_title("Euclidean Distance Error")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.grid(True)

    # Yaw error
    ax = axes[1, 1]
    ax.plot(time_axis, yaw_error)
    ax.fill_between(time_axis, 0, yaw_error, alpha=0.2, color='#F06292')
    ax.set_title("Yaw Error")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [deg]")
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.show()
    print("Saved → " + output_path)


# ────────────────────────────────────────────────────────────────────────────────
# MAIN ENTRY POINT
# ────────────────────────────────────────────────────────────────────────────────
def main():
    bag_path    = "/home/d4z/ros2/robot_sim_ws/src/robot_bag/odom_drift_1"
    output_path = "/home/d4z/ros2/robot_sim_ws/src/robot_metrics/images/plot_errors.png"

    ground_truth, odometry = read_rosbag(bag_path)

    (
        time_axis,
        ground_truth_interp,
        odometry_interp,
    ) = align_and_interpolate(ground_truth, odometry)

    (
        error_x,
        error_y,
        distance_error,
        yaw_error,
        metrics,
    ) = compute_errors(ground_truth_interp, odometry_interp)

    print(f"Duration           : {time_axis[-1]:.2f} s")
    print(f"Max distance error : {metrics['max_distance_error']:.4f} m")
    print(f"Final drift        : {metrics['final_drift']:.4f} m")
    print(f"RMSE position      : {metrics['rmse_position']:.4f} m")

    plot_results(
        time_axis,
        ground_truth_interp,
        odometry_interp,
        error_x,
        error_y,
        distance_error,
        yaw_error,
        output_path,
    )


if __name__ == "__main__":
    main()