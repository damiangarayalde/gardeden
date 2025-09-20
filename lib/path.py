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


def get_job_path(cart, start_pose=None, row_heading=0.0, sample_step=0.01,
                 margin_x=0.0, margin_y=0.0, safety_buffer=None):
    """
    Generate Dubins mowing path inside a rectangle, allowing extension (margin)
    beyond the rectangle while ensuring Dubins arcs stay inside the allowed area.

    - margin_x, margin_y: how many meters beyond rect edges you're allowed to go
      (positive -> extend outward).
    - safety_buffer: minimum inset from the allowed outer boundary to guarantee
      turn arcs fit; if None, defaults to cart.specs.turn_radius.
    """

    rect = cart.world_rect
    blade_ef_diam = cart.specs.blade_diameter
    overlap = cart.specs.overlap
    turn_radius = cart.specs.turn_radius

    x_min, y_min, x_max, y_max = rect

    # Allowed outer extents (rect expanded by margins)
    x_min_ext = x_min - margin_x
    x_max_ext = x_max + margin_x
    y_min_ext = y_min - margin_y
    y_max_ext = y_max + margin_y

    # Safety buffer (inset from the allowed extents). Default to turn radius.
    buffer = float(safety_buffer) if safety_buffer is not None else float(
        turn_radius)
    if buffer < 0:
        buffer = 0.0

    # Make sure the buffer can fit: if it's too large, reduce it to half the span
    x_span_ext = x_max_ext - x_min_ext
    y_span_ext = y_max_ext - y_min_ext

    if 2 * buffer >= x_span_ext:
        new_buf = max(0.0, (x_span_ext / 2.0) - 1e-6)
        print(
            f"Warning: horizontal safety_buffer {buffer:.3f} too large for margin. Reducing to {new_buf:.3f}")
        buffer = new_buf

    if 2 * buffer >= y_span_ext:
        new_buf = max(0.0, (y_span_ext / 2.0) - 1e-6)
        print(f"Warning: vertical safety_buffer reduced to {new_buf:.3f}")
        buffer = min(buffer, new_buf)

    # Effective endpoints (inset by buffer so turn arcs remain inside allowed outer extents)
    x_min_allowed = x_min_ext + buffer
    x_max_allowed = x_max_ext - buffer
    y_min_allowed = y_min_ext  # + buffer
    y_max_allowed = y_max_ext  # - buffer

    spacing = lane_spacing_from(blade_ef_diam, overlap)

    # If spacing makes no rows, ensure at least one centered lane
    if spacing <= 0:
        raise ValueError(
            "Invalid lane spacing (<=0). Check blade diameter/overlap.")

# Force first row at half blade above bottom
    rows = np.arange(y_min_allowed + blade_ef_diam / 2.0,
                     y_max_allowed + 1e-9,
                     spacing)

    if rows.size == 0:
        # Fallback to a single row centered in the allowed Y-range
        rows = np.array([(y_min_allowed + y_max_allowed) / 2.0])

    def lane_endpoints(y, east=True):
        # heading for east lanes is row_heading, for west lanes row_heading + pi
        if east:
            start = (x_min_allowed, y, row_heading)
            end = (x_max_allowed, y, row_heading)
        else:
            start = (x_max_allowed, y, row_heading + math.pi)
            end = (x_min_allowed, y, row_heading + math.pi)
        return start, end

    # Default start pose: leftmost allowed inset at first row
    if start_pose is None:
        start_pose = (x_min_allowed, rows[0], angle_wrap(row_heading))

    # clamp helper
    def clamp_pose(p):
        x, y, th = p
        x = min(max(x, x_min_allowed), x_max_allowed)
        y = min(max(y, y_min_allowed), y_max_allowed)
        return (x, y, angle_wrap(th))

    # ensure start_pose is inside allowed area
    start_pose = clamp_pose(start_pose)
    go_east = abs(angle_wrap(start_pose[2] - row_heading)) < 1e-6

    path_poses = []
    curr_pose = start_pose

    def connect(curr_pose, target_pose):
        seg = dubins_segment(curr_pose, target_pose,
                             turn_radius, step=sample_step)
        return seg, seg[-1]

    for i, y in enumerate(rows):
        lane_dir_east = go_east if (i % 2 == 0) else not go_east
        lane_start, lane_end = lane_endpoints(y, east=lane_dir_east)

        lane_start = (lane_start[0], lane_start[1], angle_wrap(lane_start[2]))
        lane_end = (lane_end[0], lane_end[1], angle_wrap(lane_end[2]))

        # If we're not already at lane_start, connect to it
        if (np.hypot(curr_pose[0] - lane_start[0], curr_pose[1] - lane_start[1]) > 1e-6 or
                abs(angle_wrap(curr_pose[2] - lane_start[2])) > 1e-6):
            seg, curr_pose = connect(curr_pose, lane_start)
            path_poses.extend(seg)

        # traverse the lane to its end
        seg, curr_pose = connect(curr_pose, lane_end)
        # skip duplicate start point when appending
        if len(seg) > 1:
            path_poses.extend(seg[1:])
        else:
            path_poses.extend(seg)

    return path_poses
