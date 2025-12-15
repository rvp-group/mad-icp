# Copyright 2024 R(obots) V(ision) and P(erception) group
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""MAD-ICP Map Builder Tool."""

import sys
import argparse
from pathlib import Path
from typing import Generator, Tuple, Dict
import numpy as np
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn, TimeElapsedColumn
from rich import box

from mad_icp.apps.utils.tum_reader import parse_tum_trajectory, trajectory_to_dict
from mad_icp.apps.utils.point_cloud2 import read_point_cloud

# Import pybind MADtree
from mad_icp.src.pybind.pymadtree import MADtree

console = Console()

BANNER = """
[bold cyan]  __  __    _    ____        __  __    _    ____
 |  \\/  |  / \\  |  _ \\      |  \\/  |  / \\  |  _ \\
 | |\\/| | / _ \\ | | | |_____| |\\/| | / _ \\ | |_) |
 | |  | |/ ___ \\| |_| |_____| |  | |/ ___ \\|  __/
 |_|  |_/_/   \\_\\____/      |_|  |_/_/   \\_\\_|    [/bold cyan]

[dim]Matching Adaptive Data - Map Builder[/dim]
"""


def read_clouds_from_bag(bag_path: Path, topic: str, min_range: float, max_range: float) -> Generator[Tuple[float, np.ndarray], None, None]:
    """Read point clouds from a ROS2 bag.

    Args:
        bag_path: Path to ROS2 bag directory
        topic: Topic name for point cloud messages
        min_range: Minimum range filter
        max_range: Maximum range filter

    Yields:
        (timestamp, points) tuples where points is Nx3 numpy array
    """
    try:
        from rosbags.highlevel import AnyReader
    except ModuleNotFoundError:
        console.print("[red]rosbags library not installed, run 'pip install -U rosbags'")
        sys.exit(-1)

    bag = AnyReader([bag_path])
    bag.open()

    connection = [x for x in bag.connections if x.topic == topic]
    if not connection:
        available_topics = [x.topic for x in bag.connections]
        console.print(f"[red]Topic '{topic}' not found in bag.")
        console.print(f"[yellow]Available topics: {available_topics}")
        bag.close()
        sys.exit(-1)

    msgs = bag.messages(connections=connection)

    for conn, timestamp, rawdata in msgs:
        msg = bag.deserialize(rawdata, conn.msgtype)
        cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points, _ = read_point_cloud(msg, min_range=min_range, max_range=max_range)
        yield cloud_stamp, points

    bag.close()


def count_messages_in_bag(bag_path: Path, topic: str) -> int:
    """Count the number of messages for a topic in the bag."""
    try:
        from rosbags.highlevel import AnyReader
    except ModuleNotFoundError:
        return 0

    bag = AnyReader([bag_path])
    bag.open()
    count = bag.topics.get(topic, type('', (), {'msgcount': 0})()).msgcount
    bag.close()
    return count


def write_manifest(filepath: Path, manifest: list):
    """Write manifest file mapping poses to tree files.

    Args:
        filepath: Path to manifest file
        manifest: List of (timestamp, pose_4x4, tree_filename) tuples
    """
    with open(filepath, 'w') as f:
        f.write("# MAD-ICP Map Manifest\n")
        f.write("# Format: timestamp tx ty tz qx qy qz qw tree_file\n")

        for timestamp, pose, tree_file in manifest:
            # Extract translation
            tx, ty, tz = pose[:3, 3]

            # Extract quaternion from rotation matrix
            from scipy.spatial.transform import Rotation
            rot = Rotation.from_matrix(pose[:3, :3])
            quat = rot.as_quat()  # [qx, qy, qz, qw]
            qx, qy, qz, qw = quat

            f.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} {tree_file}\n")


def print_config_table(trajectory: Path, bag: Path, topic: str, output: Path,
                       b_max: float, b_min: float, min_range: float, max_range: float,
                       num_threads: int, traj_count: int, msg_count: int):
    """Print a nice configuration table."""
    table = Table(title="Configuration", box=box.ROUNDED, show_header=False,
                  title_style="bold magenta", border_style="dim")
    table.add_column("Parameter", style="cyan")
    table.add_column("Value", style="white")

    table.add_row("Trajectory", str(trajectory))
    table.add_row("  Poses loaded", f"{traj_count}")
    table.add_row("ROS2 Bag", str(bag))
    table.add_row("  Topic", topic)
    table.add_row("  Messages", f"{msg_count}")
    table.add_row("Output", str(output))
    table.add_row("MADtree b_max", f"{b_max}")
    table.add_row("MADtree b_min", f"{b_min}")
    table.add_row("Range filter", f"[{min_range}, {max_range}] m")
    table.add_row("Threads", f"{num_threads}")

    console.print(table)
    console.print()


def print_summary(matched: int, skipped: int, output: Path, manifest_path: Path):
    """Print a nice summary panel."""
    summary = Table.grid(padding=(0, 2))
    summary.add_column(style="green")
    summary.add_column(style="white")

    summary.add_row("Matched clouds:", f"{matched}")
    summary.add_row("Skipped (no pose):", f"{skipped}")
    summary.add_row("Match rate:", f"{100*matched/(matched+skipped):.1f}%" if matched+skipped > 0 else "N/A")
    summary.add_row("Output directory:", str(output))
    summary.add_row("Manifest file:", str(manifest_path))

    console.print(Panel(summary, title="[bold green]Build Complete[/bold green]",
                       border_style="green", box=box.ROUNDED))


def main():
    """Build MAD tree database from TUM trajectory and ROS2 bag."""
    parser = argparse.ArgumentParser(
        description="Build MAD tree database from TUM trajectory and ROS2 bag for pose-graph localization.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Output structure:
    output_dir/
        trees/
            000000.bin, 000001.bin, ...
        manifest.txt
"""
    )

    parser.add_argument("-t", "--trajectory", type=Path, required=True,
                        help="Path to TUM trajectory file (timestamp tx ty tz qx qy qz qw)")
    parser.add_argument("-b", "--bag", type=Path, required=True,
                        help="Path to ROS2 bag directory")
    parser.add_argument("-T", "--topic", type=str, required=True,
                        help="Point cloud topic name (e.g., /lidar/points)")
    parser.add_argument("-o", "--output", type=Path, required=True,
                        help="Output directory for trees and manifest")
    parser.add_argument("--b-max", type=float, default=0.2,
                        help="Maximum bounding box size for MADtree leaves (default: 0.2)")
    parser.add_argument("--b-min", type=float, default=0.1,
                        help="Minimum bounding box size for normal propagation (default: 0.1)")
    parser.add_argument("--min-range", type=float, default=0.5,
                        help="Minimum point range filter in meters (default: 0.5)")
    parser.add_argument("--max-range", type=float, default=100.0,
                        help="Maximum point range filter in meters (default: 100.0)")
    parser.add_argument("-j", "--num-threads", type=int, default=4,
                        help="Number of parallel threads for tree building (default: 4)")

    args = parser.parse_args()

    # Resolve paths
    trajectory = args.trajectory.resolve()
    bag = args.bag.resolve()
    output = args.output.resolve()

    # Validate inputs
    if not trajectory.exists():
        console.print(f"[bold red]Error:[/bold red] Trajectory file not found: {trajectory}")
        sys.exit(1)
    if not bag.exists():
        console.print(f"[bold red]Error:[/bold red] Bag directory not found: {bag}")
        sys.exit(1)

    # Print banner
    console.print(BANNER)

    # Create output directories
    output.mkdir(parents=True, exist_ok=True)
    trees_dir = output / "trees"
    trees_dir.mkdir(exist_ok=True)

    # Parse trajectory with spinner
    with console.status("[bold cyan]Loading trajectory...", spinner="dots"):
        traj_list = parse_tum_trajectory(trajectory)
        traj_dict = trajectory_to_dict(traj_list)

    # Count messages
    with console.status("[bold cyan]Scanning bag...", spinner="dots"):
        total_msgs = count_messages_in_bag(bag, args.topic)

    if total_msgs == 0:
        console.print(f"[bold red]Error:[/bold red] No messages found for topic '{args.topic}'")
        console.print("[yellow]Tip: Check available topics in the bag with 'ros2 bag info <bag_path>'[/yellow]")
        sys.exit(1)

    # Print configuration
    print_config_table(trajectory, bag, args.topic, output, args.b_max, args.b_min,
                       args.min_range, args.max_range, args.num_threads, len(traj_list), total_msgs)

    # Compute max parallel level from num_threads
    import math
    max_parallel_level = int(math.log2(max(1, args.num_threads)))

    manifest = []
    matched_count = 0
    skipped_count = 0
    total_nodes = 0

    with Progress(
        SpinnerColumn(),
        TextColumn("[bold blue]{task.description}"),
        BarColumn(bar_width=40),
        TaskProgressColumn(),
        TimeElapsedColumn(),
        console=console,
        transient=False,
    ) as progress:
        task = progress.add_task("Building trees", total=total_msgs)

        for cloud_timestamp, points in read_clouds_from_bag(bag, args.topic, args.min_range, args.max_range):
            progress.advance(task)

            # Check for matching pose (with 1 microsecond tolerance)
            rounded_ts = round(cloud_timestamp, 6)
            if rounded_ts not in traj_dict:
                skipped_count += 1
                continue

            pose = traj_dict[rounded_ts]

            # Build MADtree
            tree = MADtree()
            tree.build(points, b_max=args.b_max, b_min=args.b_min, max_parallel_level=max_parallel_level)

            # Transform to global frame
            R = pose[:3, :3]
            t = pose[:3, 3]
            tree.applyTransform(R, t)

            # Serialize
            tree_filename = f"{matched_count:06d}.bin"
            tree_path = trees_dir / tree_filename
            tree.serialize(str(tree_path))

            total_nodes += tree.getNumNodes()

            # Add to manifest
            manifest.append((cloud_timestamp, pose, f"trees/{tree_filename}"))
            matched_count += 1

            # Update description with stats
            progress.update(task, description=f"Building trees [dim](matched: {matched_count})[/dim]")

    # Write manifest
    manifest_path = output / "manifest.txt"
    write_manifest(manifest_path, manifest)

    console.print()
    print_summary(matched_count, skipped_count, output, manifest_path)


if __name__ == "__main__":
    main()
