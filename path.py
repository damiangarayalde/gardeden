import math
import numpy as np
import dubins


def angle_wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def lane_spacing_from(blade_ef_diam, overlap):
    # overlap is a fraction, e.g. 0.2 -> 20% overlap
    return blade_ef_diam * (1 - overlap)


def dubins_segment(q0, q1, turn_radius, step=0.05):
    """
    q0, q1: (x, y, theta) in meters/radians
    turn_radius: meters (rho)
    step: sample distance along the Dubins curve [m]
    returns: list of (x, y, theta)
    """
    path = dubins.shortest_path(q0, q1, turn_radius)
    configs, _ = path.sample_many(step)
    return [(x, y, angle_wrap(yaw)) for (x, y, yaw) in configs]


def get_job_path(rect, blade_ef_diam, overlap, turn_radius,
                 start_pose=None, row_heading=0.0, sample_step=0.05):
    """
    rect: (x_min, y_min, x_max, y_max)
    blade_ef_diam: coverage width (m)
    overlap: fraction [0..1]
    turn_radius: min turning radius (m)
    start_pose: optional (x,y,theta). If None, start at (x_min, y_min, row_heading).
    row_heading: heading for the first lane (0 means +X / east)
    sample_step: sampling step for Dubins paths (m)

    Returns: list[(x,y,theta)] full path
    """
    x_min, y_min, x_max, y_max = rect
    spacing = lane_spacing_from(blade_ef_diam, overlap)

    # Build centerlines of each row from bottom to top
    rows = []
    y = y_min
    # Use <= so last row touches top edge if fits exactly
    while y <= y_max + 1e-9:
        rows.append(y)
        y += spacing

    if not rows:
        return []

    # First lane direction = row_heading (e.g., 0 rad for east)
    # Alternate lanes flip heading by pi
    def lane_endpoints(y, direction_east=True):
        if direction_east:
            return (x_min, y, 0.0), (x_max, y, 0.0)
        else:
            return (x_max, y, math.pi), (x_min, y, math.pi)

    # If start_pose not provided, start at beginning of first lane with row_heading
    if start_pose is None:
        if angle_wrap(row_heading) == 0.0:
            start_pose = (x_min, rows[0], 0.0)
        else:
            # Rotate lane orientation if row_heading is not 0
            # We’ll still use (east/west) geometry but set heading to row_heading
            start_pose = (x_min, rows[0], angle_wrap(row_heading))

    # Determine whether the first lane goes "east" or "west" from the start
    go_east = abs(angle_wrap(start_pose[2] - row_heading)) < 1e-6

    path_poses = []
    curr_pose = start_pose

    for i, y in enumerate(rows):
        # Which direction for this lane?
        lane_dir_east = go_east if (i % 2 == 0) else (not go_east)
        (xs, ys, hs), (xe, ye, he) = lane_endpoints(y, lane_dir_east)

        # For the very first lane, if we’re not already on its start, dubins to it
        lane_start = (xs, ys, angle_wrap(
            row_heading if lane_dir_east else row_heading + math.pi))
        if len(path_poses) == 0:
            if np.hypot(curr_pose[0] - xs, curr_pose[1] - ys) > 1e-6 or abs(angle_wrap(curr_pose[2] - lane_start[2])) > 1e-6:
                seg = dubins_segment(curr_pose, lane_start,
                                     turn_radius, step=sample_step)
                path_poses.extend(seg)
                curr_pose = seg[-1]
        else:
            # From last pose to the next lane start
            seg = dubins_segment(curr_pose, lane_start,
                                 turn_radius, step=sample_step)
            path_poses.extend(seg)
            curr_pose = seg[-1]

        # Now drive the straight lane (we still use Dubins so heading is respected)
        lane_end = (xe, ye, he)
        seg = dubins_segment(curr_pose, lane_end,
                             turn_radius, step=sample_step)
        # Skip first point to avoid duplicates
        path_poses.extend(seg[1:])
        curr_pose = seg[-1]

        # If this was the last row, we’re done (no transition to next row)
        if i == len(rows) - 1:
            break

        # Next row start heading is opposite
        # We will connect to the next lane's start in the next loop iteration

    return path_poses
