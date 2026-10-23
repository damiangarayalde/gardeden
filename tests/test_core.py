import contextlib
import io
import math
from pathlib import Path

import numpy as np
import pytest

from lib.RobotController import Robot
from lib.path import lane_spacing_from

ROOT = Path(__file__).resolve().parent.parent
DT = 1 / 20  # main loop runs at 20 FPS
CALIBRATION_POINTS = [[980, 1234], [920, 646], [1354, 617], [1561, 1125]]


@pytest.fixture
def robot():
    r = Robot(str(ROOT / "configs/robot.yml"))
    r.state.paused = False
    return r


def step(robot, n=1):
    for _ in range(n):
        robot.run_controller()
        robot.set_pose(robot.estimate_next_pose(dt=DT))


def start_tracking(robot, pose, speed):
    robot.set_pose(pose)
    with contextlib.redirect_stdout(io.StringIO()):
        robot.toggle_tracking()
    robot.state.target_speed = robot.state.speed = speed


def lane_ys(robot):
    # straight lane points have heading exactly 0 or pi
    return sorted({round(y, 6) for _, y, th in robot.job_path
                   if min(abs(th), abs(abs(th) - math.pi)) < 1e-6})


# --- Path planner -------------------------------------------------------------

def test_path_stays_inside_world(robot):
    x_min, y_min, x_max, y_max = robot.world_rect
    for x, y, _ in robot.job_path:
        assert x_min <= x <= x_max and y_min <= y <= y_max, (x, y)


def test_lanes_cover_lawn_evenly(robot):
    ys = lane_ys(robot)
    half_blade = robot.specs.blade_diameter / 2
    assert ys[0] == pytest.approx(robot.world_rect[1] + half_blade, abs=1e-6)
    assert ys[-1] == pytest.approx(robot.world_rect[3] - half_blade, abs=1e-6)
    gaps = np.diff(ys)
    assert np.allclose(gaps, gaps[0], atol=1e-5)
    assert gaps[0] <= lane_spacing_from(robot.specs.blade_diameter, robot.specs.overlap)


def test_path_has_no_gaps(robot):
    pts = np.array([(x, y) for x, y, _ in robot.job_path])
    gaps = np.hypot(*np.diff(pts, axis=0).T)
    assert gaps.max() < 0.05


# --- Kinematics / inputs --------------------------------------------------------

def test_straight_line(robot):
    robot.set_pose((1.0, 1.0, 0.0))
    robot.apply_inputs({"faster": 1})
    step(robot, 20)  # 1 s
    x, y, heading = robot.get_pose()
    assert x == pytest.approx(1.0 + robot.specs.speed_step)  # speed_step is m/s
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
    assert robot.state.angular_speed == pytest.approx(robot.specs.max_angular_speed)


def test_lateral_push_is_relative_to_heading(robot):
    robot.set_pose((1.0, 1.0, 90.0))  # facing +y, so "left" is -x
    robot.apply_lateral_disturbance(0.2)
    x, y, _ = robot.get_pose()
    assert (x, y) == pytest.approx((0.8, 1.0))


def test_joystick_axes_only_sent_on_change():
    from lib.UserInputManager import UserInputManager

    class FakeStick:
        axes = [0.0, 0.0, 0.0, 0.0]
        def get_axis(self, i):
            return self.axes[i]

    ui = UserInputManager()
    ui.joystick = FakeStick()
    assert ui._joystick_axes_to_kv()        # first poll reports the state
    assert ui._joystick_axes_to_kv() == {}  # unchanged: don't override the keyboard
    ui.joystick.axes = [0.0, 0.0, 0.0, -1.0]
    assert ui._joystick_axes_to_kv()["speed_percent"] == 255


# --- Controller ------------------------------------------------------------------

def test_pure_pursuit_recovers_from_push(robot):
    lane_y = lane_ys(robot)[0]  # first lane, driven east
    robot.covered[:np.argmax(robot._path_xy[:, 0] >= 1.0)] = True  # lane already mowed up to x=1
    start_tracking(robot, (1.0, lane_y, 0.0), speed=0.4)
    robot.apply_lateral_disturbance(0.2)
    step(robot, 40)  # 2 s
    assert robot.get_pose()[1] == pytest.approx(lane_y, abs=0.02)


@pytest.mark.parametrize("speed", [0.2, 0.4, 0.6])  # 0.6 relies on slowing for curves
def test_completes_whole_job_on_path(robot, speed):
    x0, y0, th0 = robot.job_path[0]
    start_tracking(robot, (x0, y0, math.degrees(th0)), speed)
    max_err, steps = 0.0, 0
    with contextlib.redirect_stdout(io.StringIO()):
        while robot.state.tracking_enabled and steps < 5000:
            step(robot)
            steps += 1
            max_err = max(max_err, np.hypot(*(robot._path_xy - robot.get_pose()[:2]).T).min())
    assert not robot.state.tracking_enabled, "never reached the end of the job"
    assert robot.state.speed == 0
    assert max_err < 0.1
    assert robot.covered.all(), f"{(~robot.covered).sum()} waypoints left uncut"


def test_blade_marks_waypoints_covered(robot):
    x, y, _ = robot.job_path[500]
    assert not robot.covered[500]
    robot.set_pose((x, y, 0.0))  # e.g. driven there manually
    assert robot.covered[500]


def test_resume_returns_to_first_uncut_waypoint(robot):
    x0, y0, th0 = robot.job_path[0]
    start_tracking(robot, (x0, y0, math.degrees(th0)), speed=0.4)
    with contextlib.redirect_stdout(io.StringIO()):
        step(robot, 60)                        # mow for 3 s
        resume = robot.first_uncovered_index()
        robot.toggle_tracking()                # take over manually...
        robot.set_pose((2.5, 1.2, 90.0))       # ...and leave the robot somewhere else
        robot.toggle_tracking()                # back to auto
    robot.state.target_speed = robot.state.speed = 0.4
    assert robot.approach_path is not None
    rx, ry, _ = robot.job_path[resume]
    max_omega = robot.state.speed / robot.specs.turn_radius  # tightest physical turn
    closest = math.inf
    for _ in range(300):
        robot.run_controller()
        omega = (robot.state.v_right - robot.state.v_left) / robot.specs.distance_between_wheels
        # regression: the approach→path handoff used to spike omega to ~200 rad/s (heading flip)
        assert abs(omega) <= max_omega + 1e-9
        robot.set_pose(robot.estimate_next_pose(dt=DT))
        closest = min(closest, math.hypot(robot.state.x - rx, robot.state.y - ry))
    assert closest < 0.02, "never came back to where mowing stopped"
    assert robot.approach_path is None and robot.current_path_index > resume


# --- Homography ------------------------------------------------------------------

def test_homography_round_trip(robot):
    from lib.RealWorldObserver import Observer
    obs = Observer(robot.world_rect, str(ROOT / "snapshot.jpg"), CALIBRATION_POINTS)
    for world_pt in [(0.0, 0.0), (1.75, 1.05), (3.5, 2.1)]:
        # world_to_image rounds to whole pixels, so allow ~1 px of slack in meters
        assert obs.image_to_world(obs.world_to_image(world_pt)) == pytest.approx(world_pt, abs=0.02)


def test_observer_rejects_wrong_point_count(robot):
    from lib.RealWorldObserver import Observer
    with pytest.raises(ValueError):
        Observer(robot.world_rect, str(ROOT / "snapshot.jpg"), CALIBRATION_POINTS[:3])


def test_no_reversing_in_auto_mode(robot):
    start_tracking(robot, robot.get_pose(), speed=0.0)
    robot.apply_inputs({"speed_percent": -255})
    assert robot.state.speed == 0
    robot.apply_inputs({"slower": 1})
    assert robot.state.speed == 0


def test_search_window_follows_waypoint_spacing(robot):
    from lib.path import get_job_path
    for sample_step in (0.01, 0.02, 0.05):
        robot.set_job_path(get_job_path(robot, sample_step=sample_step,
                                        safety_buffer=robot.specs.turn_radius + 0.2))
        window_m = robot._search_window * sample_step
        assert window_m == pytest.approx(1.0, abs=2 * sample_step)  # ceil + sampling jitter


def lawn_coverage(world_rect, blade_xy, blade_diameter, res=0.02):
    """Fraction of the lawn (on a res grid) within half a blade of any blade position."""
    x0, y0, x1, y1 = world_rect
    gx, gy = np.meshgrid(np.arange(x0 + res / 2, x1, res), np.arange(y0 + res / 2, y1, res))
    cells = np.c_[gx.ravel(), gy.ravel()]
    cut = np.zeros(len(cells), bool)
    pts = np.asarray(blade_xy)[::2]
    for chunk in np.array_split(pts, max(1, len(pts) // 200)):
        cut |= (((cells[:, None] - chunk[None]) ** 2).sum(-1) <= (blade_diameter / 2) ** 2).any(1)
    return cut.mean()


def test_planned_path_cuts_whole_lawn_but_corners(robot):
    # Only the 4 corners stay uncut: a 0.2 m turn radius can't square them off
    xy = [(x, y) for x, y, _ in robot.job_path]
    assert lawn_coverage(robot.world_rect, xy, robot.specs.blade_diameter) > 0.99


def test_auto_mode_slows_for_curves(robot):
    # Job starts along the perimeter's bottom side, then turns up the first corner
    x0, y0, th0 = robot.job_path[0]
    start_tracking(robot, (x0, y0, math.degrees(th0)), speed=0.6)
    speeds, headings = [], []
    for _ in range(120):  # 6 s: the ~2.75 m straight, then the corner
        step(robot)
        speeds.append(robot.state.speed)
        headings.append(robot.get_pose()[2])
    in_corner = [v for v, h in zip(speeds, headings) if 20 < h < 70]
    assert max(speeds) == pytest.approx(0.6)  # full speed on the straight
    assert in_corner and max(in_corner) < 0.6 * 0.8  # clearly slower through the corner
