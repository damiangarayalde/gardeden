import math
import numpy as np
import dubins


def angle_wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def lane_spacing_from(blade_ef_diam, overlap):
    # overlap is a fraction, e.g. 0.2 -> 20% overlap
    return blade_ef_diam * (1 - overlap)


def dubins_segment(q0, q1, turn_radius, step=0.5):
    """
    q0, q1: (x, y, theta) in meters/radians
    turn_radius: meters (rho)
    step: sample distance along the Dubins curve [m]
    returns: list of (x, y, theta)
    """
    path = dubins.shortest_path(q0, q1, turn_radius)
    configs, _ = path.sample_many(step)
    return [(x, y, angle_wrap(yaw)) for (x, y, yaw) in configs]


def get_job_path(cart, start_pose=None, row_heading=0.0, sample_step=0.01):
    rect = cart.world_rect
    blade_ef_diam = cart.specs.blade_diameter
    overlap = cart.specs.overlap
    turn_radius = cart.specs.turn_radius

    x_min, y_min, x_max, y_max = rect
    spacing = lane_spacing_from(blade_ef_diam, overlap)

    rows = np.arange(y_min, y_max + 1e-9, spacing)
    if rows.size == 0:
        return []

    def lane_endpoints(y, east=True):
        return ((x_min, y, 0.0), (x_max, y, 0.0)) if east else \
               ((x_max, y, math.pi), (x_min, y, math.pi))

    if start_pose is None:
        start_pose = (x_min, rows[0], angle_wrap(row_heading))

    go_east = abs(angle_wrap(start_pose[2] - row_heading)) < 1e-6

    path_poses = []
    curr_pose = start_pose

    def connect(curr_pose, target_pose):
        seg = dubins_segment(curr_pose, target_pose,
                             turn_radius, step=sample_step)
        return seg, seg[-1]

    for i, y in enumerate(rows):
        lane_dir_east = go_east if (i % 2 == 0) else not go_east
        (x_start, y_start, h_start), (x_end, y_end,
                                      h_end) = lane_endpoints(y, lane_dir_east)

        lane_heading = row_heading if lane_dir_east else row_heading + math.pi
        lane_start = (x_start, y_start, angle_wrap(lane_heading))

        if (np.hypot(curr_pose[0] - x_start, curr_pose[1] - y_start) > 1e-6 or
                abs(angle_wrap(curr_pose[2] - lane_start[2])) > 1e-6):
            seg, curr_pose = connect(curr_pose, lane_start)
            path_poses.extend(seg)

        lane_end = (x_end, y_end, h_end)
        seg, curr_pose = connect(curr_pose, lane_end)
        path_poses.extend(seg[1:])  # skip duplicate start point

    return path_poses
