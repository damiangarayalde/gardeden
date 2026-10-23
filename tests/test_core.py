import math
from pathlib import Path

import numpy as np
import pytest

from lib.RobotController import Robot
from lib.path import lane_spacing_from

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def robot():
    r = Robot(str(ROOT / "configs/robot.yml"))
    r.state.paused = False
    return r


def step(robot, n=1):
    for _ in range(n):
        robot.run_controller()
        robot.set_pose(robot.estimate_next_pose(dt=1.0))


# --- Path planner -------------------------------------------------------------

@pytest.mark.xfail(strict=True, reason="#8: top-lane Dubins turn overshoots world.rect")
def test_path_stays_inside_world(robot):
    x_min, y_min, x_max, y_max = robot.world_rect
    for x, y, _ in robot.job_path:
        assert x_min <= x <= x_max and y_min <= y <= y_max, (x, y)


def test_lanes_evenly_spaced(robot):
    # straight lane points have heading exactly 0 or pi
    lane_ys = sorted({round(y, 6) for _, y, th in robot.job_path
                      if min(abs(th), abs(abs(th) - math.pi)) < 1e-6})
    expected = lane_spacing_from(robot.specs.blade_diameter, robot.specs.overlap)
    assert len(lane_ys) > 1
    assert np.allclose(np.diff(lane_ys), expected)


def test_path_has_no_gaps(robot):
    pts = np.array([(x, y) for x, y, _ in robot.job_path])
    gaps = np.hypot(*np.diff(pts, axis=0).T)
    assert gaps.max() < 0.05


# --- Kinematics / inputs --------------------------------------------------------

def test_straight_line(robot):
    robot.set_pose((1.0, 1.0, 0.0))
    robot.apply_inputs({"faster": 1})
    step(robot, 10)
    x, y, heading = robot.get_pose()
    assert x == pytest.approx(1.0 + 10 * robot.specs.speed_step)
    assert y == pytest.approx(1.0) and heading == pytest.approx(0.0)


def test_pause_stops_all_motion(robot):
    robot.apply_inputs({"faster": 1, "turn_left": 1})
    robot.apply_inputs({"toggle_pause": 1})
    before = robot.get_pose()
    step(robot, 10)
    assert robot.get_pose() == pytest.approx(before)


def test_idle_robot_does_not_grow_walked_path(robot):
    step(robot, 10)
    assert len(robot.get_walked_path()) == 1


def test_steering_only_analog_input(robot):
    robot.apply_inputs({"omega_percent": 255})
    assert robot.state.angular_speed != 0


# --- Controller ------------------------------------------------------------------

def test_pure_pursuit_recovers_from_push(robot):
    lane_y = min(y for _, y, th in robot.job_path if abs(th) < 1e-6)  # first east lane
    robot.set_pose((1.0, lane_y, 0.0))
    robot.toggle_tracking()
    robot.apply_inputs({"faster": 1})
    robot.apply_inputs({"faster": 1})
    robot.apply_lateral_disturbance(0.1)
    step(robot, 50)
    assert robot.get_pose()[1] == pytest.approx(lane_y, abs=0.02)


# --- Homography ------------------------------------------------------------------

def test_homography_round_trip():
    from lib.RealWorldObserver import Observer
    obs = Observer(snapshot_path=str(ROOT / "snapshot.jpg"),
                   predefined_points=[[980, 1234], [920, 646], [1354, 617], [1561, 1125]])
    for world_pt in [(0.0, 0.0), (1.75, 1.05), (3.5, 2.1)]:
        # world_to_image rounds to whole pixels, so allow ~1 px of slack in meters
        assert obs.image_to_world(obs.world_to_image(world_pt)) == pytest.approx(world_pt, abs=0.02)
