import numpy as np


def get_transformed_pose(lidar_to_world, lidar_to_base) -> np.ndarray:
	base_to_lidar = np.linalg.inv(lidar_to_base)
	base_to_world = np.dot(lidar_to_base, np.dot(lidar_to_world, base_to_lidar))
	return base_to_world


def write_transformed_pose(estimate_file, base_to_world) -> None:
	np.savetxt(estimate_file, base_to_world[:3].reshape(-1, 12))
