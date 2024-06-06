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

import numpy as np
import open3d as o3d
from matplotlib import cm
from matplotlib.colors import Normalize

GREEN = np.array([0, 255, 0]) / 255.0
BLACK = np.array([0, 0, 0]) / 255.0
WHITE = np.array([255, 255, 255]) / 255.0
SPHERE_SIZE = 0.20

class Visualizer():
	def __init__(self):
		self.pause = True
		self.reset_view_point = True

		self.current = o3d.geometry.PointCloud()
		self.local_map = o3d.geometry.PointCloud()
		self.trajectory_points = []

		self.vis = o3d.visualization.VisualizerWithKeyCallback()
		self._register_key_callbacks()
		self._initialize_visualizer()

	def _initialize_visualizer(self):
		self.vis.create_window()
		self.vis.add_geometry(self.current)
		self.vis.add_geometry(self.local_map)
		self.vis.get_render_option().background_color = BLACK
		self.vis.get_render_option().point_size = 1
		print("Visualizer commands:\n"
			"\t[SPACE] to pause/start\n"
			"\t    [X] to center the viewpoint\n")

	def update(self, current, local_map, pose, kf):
		self._update_geometries(current, local_map, pose, kf)
		while True:
			self.vis.poll_events()
			self.vis.update_renderer()
			if not self.pause:
				break

	def _register_key_callback(self, key, callback):
		self.vis.register_key_callback(ord(key), callback)

	def _register_key_callbacks(self):
		self._register_key_callback(" ", self._pause)
		self._register_key_callback("X", self._reset_view_point)

	def _pause(self, vis):
		self.pause = not self.pause

	def _reset_view_point(self, vis):
		self.reset_view_point = True

	def _update_geometries(self, current, local_map, pose, kf):
		self.current.points = o3d.utility.Vector3dVector(current)
		self.current.paint_uniform_color(WHITE)
		self.vis.update_geometry(self.current)

		if local_map is not None:
			self.local_map.points = o3d.utility.Vector3dVector(local_map)
			colors = cm.plasma(Normalize()(np.asarray(local_map)[:,2]))[:, :3]
			self.local_map.colors = o3d.utility.Vector3dVector(colors)
			self.vis.update_geometry(self.local_map)

		self.trajectory_points.append(pose[:3, 3].tolist())
		if len(self.trajectory_points) > 1:
			points = np.array([self.trajectory_points[-2], self.trajectory_points[-1]])
			trajectory_line = o3d.geometry.LineSet()
			trajectory_line.points = o3d.utility.Vector3dVector(points)
			trajectory_line.lines = o3d.utility.Vector2iVector([[0, 1]])
			trajectory_line.paint_uniform_color(GREEN)
			self.vis.add_geometry(trajectory_line, reset_bounding_box=False)

		if kf is not None:
			sphere = o3d.geometry.TriangleMesh.create_sphere(SPHERE_SIZE)
			sphere.paint_uniform_color(GREEN)
			sphere.transform(kf)
			self.vis.add_geometry(sphere, reset_bounding_box=False)

		if self.reset_view_point:
			self.vis.reset_view_point(True)
			self.reset_view_point = False