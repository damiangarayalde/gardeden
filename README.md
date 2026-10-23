# Gardeden

[![tests](https://github.com/damiangarayalde/gardeden/actions/workflows/tests.yml/badge.svg)](https://github.com/damiangarayalde/gardeden/actions/workflows/tests.yml)

**An autonomous lawn-mowing robot, built step by step: simulation first, then the real thing.**

Gardeden is a personal training project. After some time away from robotics I wanted to get back into it with a hands-on robotics and simulation build, so I picked a task I do every week: mowing the lawn. The goal is a small robotic mower that cuts the lawn by itself, follows an efficient coverage path, and hands control back to a person at any moment.

Before putting blades on real hardware, I wanted a setup where I could design and test the mowing logic safely. So the first milestone is a simulator that projects a virtual mower onto a real photo of my garden. I drive it with a game controller and switch it in and out of autonomous mode.

This is an exploration project. It isn't trying to fix a particular bottleneck or beat an existing product. It's about understanding every piece of the system by building it myself.

<!-- Demo video: on github.com, edit this README and drag the .mp4 file here. GitHub uploads it and inserts a link that plays inline. -->
> 🎥 **Demo:** *(video coming soon)* Me driving the mower with a joystick over the camera view of the garden, then switching to auto mode so it follows the planned mowing path on its own.

---

## Why this project

Mowing is a good robotics problem because it's simple to describe and hard to do well:

- **Coverage:** every part of the lawn should get cut, with as little overlap and wasted travel as possible.
- **Real vehicle constraints:** a mower can't turn on the spot like a cursor. It has a minimum turning radius, and the path has to respect it.
- **Localization:** the robot has to know where it is in the garden, not only where it thinks it is.
- **Human in the loop:** a person should be able to take over instantly, and auto mode should resume cleanly afterwards.

Gardeden is my way of working through each of these, end to end, on a project I can actually see working in my own garden.

## Design choices

**No robotics simulation suite, on purpose.** I left out tools like Gazebo or Isaac Sim. The current robot model is simple enough that they would add more setup than value. I'd rather keep the system small and modular: each block (input, control, kinematics, path planning, rendering, comms) is a plain Python module I can read, test and replace on its own.

**Full ownership of the stack.** I wanted complete control over the communication, the data and the closed-loop control. If this ever grows into something more professional, I'll know exactly why each block exists and what it does, instead of inheriting decisions from a framework.

**From 2D, to camera projection, to (optionally) vision.** The project started as a purely 2D top-down simulation. The next step was projecting it onto a real image of the garden. That step brings both me and the software closer to the real scene, and it sets up an optional closed loop based on vision. I briefly looked at other ways to localize the robot. I found vision the most elegant: it needs no extra hardware, since a security camera already covers the garden, and it's closer to how an independent robot would be expected to perceive the world.

## What works today

| Area | What it does | Where |
|---|---|---|
| **Coverage path planning** | Generates a back-and-forth ("boustrophedon") mowing pattern over the lawn. Lane spacing comes from the blade diameter and a configurable overlap, and the turns between lanes are joined with **Dubins curves** so the path respects the mower's minimum turning radius. The job starts with a lap around the edge, which cuts the strips at the lane ends where the turns happen. With perfect tracking it covers 99.2% of the lawn; only the four corners stay uncut, because a 0.2 m turning radius can't square them off. | [lib/path.py](lib/path.py) |
| **Path following** | A **pure-pursuit** controller with a speed-dependent lookahead steers the robot along the planned path. It only searches about 1 m forward along the path, so a sideways push can't make it skip to the next lane. Steering is capped at the mower's minimum turning radius. The speed I set is a ceiling: the controller slows down for curves to keep cornering under `max_lateral_accel`. When nothing uncut is left ahead, or reaching it would mean re-driving already-cut lane, the robot heads for the next uncut stretch. That covers grass missed after a push, and it skips grass I already cut by hand. If it comes back for a patch and still misses it, it gives up on that patch and reports it, instead of circling forever. | [lib/RobotController.py](lib/RobotController.py) |
| **Robot model** | Differential-drive kinematics: linear and angular velocity in, left/right wheel speeds and a new pose out. It also records the area already cut: every planned waypoint the blade passes over (within 10 cm), whether in manual or auto mode, is marked as covered. | [lib/RobotController.py](lib/RobotController.py) |
| **Manual / auto switching** | Drive manually with a joystick or keyboard, then toggle auto mode. In auto mode the controller does the steering and I still set the speed (forward only, since pure pursuit can't steer in reverse). You can pause at any time or push the robot sideways to check that the controller recovers. When auto mode comes back on, the robot resumes at the **first uncut waypoint**. If that waypoint is far away, it plans a Dubins transit there (drawn in yellow), so it arrives lined up with the lane instead of cutting in partway. | [lib/UserInputManager.py](lib/UserInputManager.py) |
| **Camera view (AR overlay)** | A homography, calibrated from 4 points on a real photo of the garden, maps lawn coordinates in meters to camera pixels. The planned path, the cut area and the robot are drawn in perspective on top of the actual image. | [lib/RealWorldObserver.py](lib/RealWorldObserver.py), [lib/draw.py](lib/draw.py) |
| **Visualization** | Top-down ("zenital") view, camera view, or both side by side. | [lib/draw.py](lib/draw.py) |
| **Robot comms (groundwork)** | A UDP/JSON link to an ESP32 on the robot that sends the same control inputs the simulator uses. Off by default (`enabled` in `configs/comms.yml`). | [lib/CommModule.py](lib/CommModule.py) |

## How it fits together

Everything runs in one loop in [main.py](main.py), at 20 Hz:

```
 joystick / keyboard ──► UserInputManager ──► inputs {speed, steering, toggles}
                                                    │
                                                    ▼
                         Robot.apply_inputs()  (manual commands)
                         Robot.run_controller() (pure pursuit, if auto mode is on)
                         Robot.estimate_next_pose() (differential-drive model)
                                                    │
                                                    ▼
                         Drawer.draw_scene() ──► top-down view and/or
                                                 camera view via Observer homography
```

All inputs travel as a small key/value dict (for example `{"speed_percent": 120, "toggle_tracking": 1}`). I did that on purpose: the dict the simulator consumes is the same message that `CommModule` sends to the ESP32. Moving from the simulation to the real robot should mean changing where the inputs go, not rewriting the control logic.

## Running it

Requires [uv](https://docs.astral.sh/uv/) (`brew install uv`) and, optionally, a game controller. I use a PlayStation-style gamepad. uv installs the right Python (3.11, from `.python-version`) and all dependencies into `.venv`.

```sh
uv sync                 # create/update .venv from pyproject.toml + uv.lock
uv run main.py          # run from the repo root, since config paths are relative
uv run pytest           # run the tests (CI runs the same on every push: .github/workflows/tests.yml)
```

`uv run` always uses the project's `.venv`, so there's nothing to activate. If you prefer, `source .venv/bin/activate` and then plain `python main.py` / `pytest` work too. To add a dependency, use `uv add <package>` (or `uv add --group dev <package>` for dev-only tools). For the notebooks in `experiments/`, run `uv sync --group notebooks`.

By default the app opens in camera mode, using the snapshot and calibration points in `configs/camera.yml`. To recalibrate against a different photo, set `calibration_points: null`, then click the 4 lawn corners on the image (in the order listed in that file) and press **Enter**. The console prints the points so you can paste them back into the config.

To switch between the top-down view, the camera view, or both side by side, change `camera_mode=` in `main.py` to `"zenital"`, `"camera"` or `"both"`.

### Controls

The robot starts **paused**. Press Start (or `P`) to begin.

| Action | Joystick | Keyboard |
|---|---|---|
| Pause / resume | Start | `P` |
| Toggle auto mode (path following; resumes at the first uncut waypoint) | Options | `T` |
| Speed | Right stick ↕ | `W` / `S` |
| Steering | Right stick ↔ | `A` / `D` |
| Lateral push (disturbance test) | L1 / R1 | `Z` / `C` |
| Horn / blade indicator | Square | — |

Button numbers can differ between controllers. If yours doesn't match, adjust the `JS_*` constants at the top of [lib/UserInputManager.py](lib/UserInputManager.py).

### Configuration

| File | What you can tune |
|---|---|
| [configs/robot.yml](configs/robot.yml) | Lawn size (`world.rect`, in meters), blade diameter, wheelbase, turning radius, lane overlap, keyboard step sizes and joystick max speeds (m/s, rad/s), the cornering limit for auto mode (`max_lateral_accel`), and whether the robot starts paused or in auto mode. |
| [configs/camera.yml](configs/camera.yml) | Garden snapshot and the 4 calibration points (lawn corners in image pixels). |
| [configs/graphics.yml](configs/graphics.yml) | Window size. |
| [configs/comms.yml](configs/comms.yml) | Whether to send inputs to the robot, plus the ESP32 IP address and UDP port. |

## Repository layout

```
main.py                  main loop: inputs → control → model → render
lib/
  RobotController.py     robot specs, state, kinematics, pure pursuit
  path.py                coverage path planner (boustrophedon + Dubins)
  UserInputManager.py    joystick + keyboard → input dict
  RealWorldObserver.py   camera calibration and homography (world ↔ image)
  draw.py                rendering: top-down, camera, split-screen
  CommModule.py          UDP/JSON link to the robot's ESP32
configs/                 YAML configuration
pyproject.toml, uv.lock  dependencies (managed with uv)
snapshot.jpg             reference photo of the garden used for the camera view
tests/                   pytest suite (`uv run pytest`)
experiments/             ArUco marker detection, notebooks
```

## Roadmap

Roughly in order:

1. **Close the loop on hardware.** Send the input dict to the ESP32 (already wired: set `enabled: true` in `configs/comms.yml`) and drive the real chassis from the same joystick.
2. **Live camera instead of a snapshot.** Swap the static `snapshot.jpg` for the live feed from the security camera that already watches the garden. I've confirmed I can reach it from custom software.
3. **Vision-based localization (optional closed loop).** Detect ArUco markers on the robot (early experiments are in [experiments/](experiments/)) and feed the measured pose back into the controller, so it no longer relies only on dead reckoning.
4. **Real lawn shapes.** Handle non-rectangular areas and obstacles, not only a rectangle.
5. **Safety.** Blade cutoff, a communication-loss failsafe, and keeping the robot inside the lawn boundary.

## For collaborators

- **Start with [main.py](main.py).** It's short and shows the whole loop. From there, [lib/RobotController.py](lib/RobotController.py) is the core.
- **Units:** meters and seconds in the world, and degrees for the robot heading. The path planner works in radians internally.
- **Adding an input:** emit a new key in `UserInputManager`, then handle it in `Robot.apply_inputs()`. Since the robot reads the same dict, that key will also work over UDP once comms are live.
- **Tuning:** most behavior is in `configs/robot.yml`, so try changing it there before editing code.
- **Branches:** `main` should always run. Please put experiments on a branch or under `experiments/`.

## Tech

Python · Pygame (rendering and controller input) · OpenCV (homography, ArUco) · NumPy · [pydubins](https://github.com/rdesc/pydubins) (Dubins paths) · ESP32 over UDP

---

*Built by [Damian Garayalde](https://github.com/damiangarayalde) as a training and exploration project. If you'd like to modify it, improve it or build on top of it, please get in touch first. I'm happy to talk about it.*
