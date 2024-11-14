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
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import typer
from typing_extensions import Annotated

# binded vectors and madtree
from mad_icp.src.pybind.pyvector import VectorEigen3d
from mad_icp.src.pybind.pymadicp import MADicp
from mad_icp.src.pybind.pymadtree import MADtree

from mad_icp.apps.utils.tools.tools_utils import generate_four_walls_pointcloud


MAX_ITERATIONS = 15
app = typer.Typer()
T_guess = np.eye(4)

def main(viz: Annotated[bool, typer.Option(help="if true visualizer on (very slow)", show_default=True)] = False) -> None:

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

    # create figure and 3d axis for visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ref_scatter = ax.scatter(ref_cloud[:, 0], ref_cloud[:, 1], ref_cloud[:, 2], color='blue', s=1, label='reference cloud')
    query_cloud_transformed = np.array([T_guess[:3, :3] @ point + T_guess[:3, 3] for point in query_cloud])
    query_scatter = ax.scatter(query_cloud_transformed[:, 0], query_cloud_transformed[:, 1], query_cloud_transformed[:, 2], color='red', s=1, label='query cloud')

    # set axis limits
    ax.legend()

    # this is just to visualize convergence of ICP
    # and matches during iterations
    tree = MADtree()
    tree.build(VectorEigen3d(ref_cloud))

    # function to update the animation for each iteration
    def update(iteration):
        global T_guess
        # perform one iteration of icp
        T_guess = madicp.compute(T_guess, icp_iterations=1)
        # transform the query cloud with the current estimate of the transformation
        query_cloud_transformed = np.array([T_guess[:3, :3] @ point + T_guess[:3, 3] for point in query_cloud])
        ref_cloud_matched = tree.searchCloud(VectorEigen3d(query_cloud_transformed))
        ref_cloud_points = np.array([ref_point_matched for ref_point_matched, _ in ref_cloud_matched])

        # remove existing lines from plot
        [line.remove() for line in ax.lines]

        # draw lines between each matched point pair
        [ax.plot([qp[0], rp[0]], [qp[1], rp[1]], [qp[2], rp[2]], color='green', linestyle='-', linewidth=0.2) 
            for qp, rp in zip(query_cloud_transformed, ref_cloud_points)]
            
        # update the query cloud scatter plot data
        query_scatter._offsets3d = (query_cloud_transformed[:, 0], query_cloud_transformed[:, 1], query_cloud_transformed[:, 2])
        ax.set_title(f"MAD registration: iteration {iteration+1}")

    # create animation object, updating for each iteration
    ani = FuncAnimation(fig, update, frames=MAX_ITERATIONS, interval=0, repeat=False)

    # display the animation
    plt.show()

def run():
    typer.run(main)

if __name__ == '__main__':
    run()

