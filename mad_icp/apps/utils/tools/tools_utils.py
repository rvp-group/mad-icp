import numpy as np

def generate_four_walls_pointcloud(wall_height=2.0, wall_width=4.0, points_per_wall=10000):
    # helper function to generate points for each plane
    def generate_plane(x_range, y_range, z_range, points):
        x = np.random.uniform(x_range[0], x_range[1], points)
        y = np.random.uniform(y_range[0], y_range[1], points)
        z = np.random.uniform(z_range[0], z_range[1], points)
        return np.column_stack((x, y, z))

    # generate points for each wall
    wall1 = generate_plane([0, wall_width], [0, 0], [0, wall_height], points_per_wall)  # wall along Y=0
    wall2 = generate_plane([0, wall_width], [wall_width, wall_width], [0, wall_height], points_per_wall)  # wall along Y=wall_width
    wall3 = generate_plane([0, 0], [0, wall_width], [0, wall_height], points_per_wall)  # wall along X=0
    wall4 = generate_plane([wall_width, wall_width], [0, wall_width], [0, wall_height], points_per_wall)  # wall along X=wall_width
    
    # generate points for the floor
    floor = generate_plane([0, wall_width], [0, wall_width], [0, 0], points_per_wall)  # floor at Z=0

    # combine all walls and the floor into a single point cloud
    return np.vstack((wall1, wall2, wall3, wall4, floor))