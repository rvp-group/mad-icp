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

import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import typer
from typing_extensions import Annotated
import time

# binded vectors and madtree
from mad_icp.src.pybind.pyvector import VectorEigen3d
from mad_icp.src.pybind.pymadicp import MADicp
from mad_icp.src.pybind.pymadtree import MADtree

from mad_icp.apps.utils.tools.tools_utils import generate_four_walls_pointcloud

MAX_ITERATIONS = 15
app = typer.Typer()
T_guess = np.eye(4)

def main(viz: Annotated[bool, typer.Option(help="if true visualizer on", show_default=True)] = False) -> None:

    # initialize reference and query clouds
    np.random.seed(42)
    ref_cloud = generate_four_walls_pointcloud(points_per_wall=1000)
    query_cloud = ref_cloud.copy()

    # initial transformation guess
    global T_guess
    T_guess[:3, :3] = R.from_euler('xyz', [0.1, 0.1, 0.1]).as_matrix()
    T_guess[:3, 3] = np.random.rand(3)
    print("init guess T\n", T_guess)
    print("gt T\n", np.eye(4))

    # initialize mad icp object and set reference and query clouds
    madicp = MADicp(num_threads=os.cpu_count())
    madicp.setReferenceCloud(VectorEigen3d(ref_cloud))
    madicp.setQueryCloud(VectorEigen3d(query_cloud))
    if not viz:
        T_est = madicp.compute(T_guess, icp_iterations=MAX_ITERATIONS)
        print("estimate \n", T_est)
        exit(0)

    # create open3d point clouds for visualization
    ref_pcd = o3d.geometry.PointCloud()
    ref_pcd.points = o3d.utility.Vector3dVector(ref_cloud)
    ref_pcd.paint_uniform_color([0, 0, 1])  # blue

    query_pcd = o3d.geometry.PointCloud()
    query_cloud_transformed = np.array([T_guess[:3, :3] @ point + T_guess[:3, 3] for point in query_cloud])
    query_pcd.points = o3d.utility.Vector3dVector(query_cloud_transformed)
    query_pcd.paint_uniform_color([1, 0, 0])  # red

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="MAD-registration", width=1280, height=720, visible=True)
    vis.add_geometry(ref_pcd)
    vis.add_geometry(query_pcd)

    # this is just to visualize convergence of ICP
    # and matches during iterations
    tree = MADtree()
    tree.build(VectorEigen3d(ref_cloud))

    # function to update the visualization for each iteration
    def update(iteration):
        global T_guess
        # perform one iteration of icp
        T_guess = madicp.compute(T_guess, icp_iterations=1)
        # transform the query cloud with the current estimate of the transformation
        query_cloud_transformed = np.array([T_guess[:3, :3] @ point + T_guess[:3, 3] for point in query_cloud])
        ref_cloud_matched = tree.searchCloud(VectorEigen3d(query_cloud_transformed))
        ref_cloud_points = np.array([ref_point_matched for ref_point_matched, _ in ref_cloud_matched])

        # update the query cloud point cloud data
        query_pcd.points = o3d.utility.Vector3dVector(query_cloud_transformed)
        vis.update_geometry(query_pcd)
        vis.poll_events()
        vis.update_renderer()
        vis.get_render_option().point_size = 5
        vis.get_render_option().background_color = np.asarray([1, 1, 1])
        vis.get_render_option().show_coordinate_frame = False
        vis.get_render_option().line_width = 2.0

        # draw lines between each matched point pair
        lines = [[i, i + len(query_cloud_transformed)] for i in range(len(query_cloud_transformed))]
        colors = [[0, 1, 0] for _ in range(len(query_cloud_transformed))]  # green
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.vstack((query_cloud_transformed, ref_cloud_points))),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        vis.add_geometry(line_set)
        vis.poll_events()
        vis.update_renderer()
        return line_set

    # update the visualization for each iteration
    line_set = None
    for iteration in range(MAX_ITERATIONS):
        if line_set is not None:
            vis.remove_geometry(line_set)
        line_set = update(iteration)
        
        time.sleep(0.1)  # sleep for 100 ms

    vis.run()
    vis.destroy_window()

def run():
    typer.run(main)

if __name__ == '__main__':
    run()