import pygame
from typing import Dict, Any, Tuple

JS_BTN_OPTIONS = 6
JS_BTN_START = 5
JS_BTN_L1 = 10
JS_BTN_R1 = 9
JS_BTN_SQUARE = 2
JS_AXIS_LEFT_HORIZONTAL = 0
JS_AXIS_LEFT_VERTICAL = 1
JS_AXIS_RIGHT_HORIZONTAL = 2
JS_AXIS_RIGHT_VERTICAL = 3


class UserInputManager:
    """Centralizes keyboard and joystick inputs into a single key/value dict."""

    def __init__(self, deadzone: float = 0.2, debug: bool = False):
        self.deadzone = deadzone
        self.debug = debug

        # ----------------------------------------------------
        # Try initialize joystick

        pygame.joystick.init()
        self.joystick = None
        self._last_axes_kv = None

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick detected: {self.joystick.get_name()}")
        else:
            print("⚠️ No joystick detected. Keyboard only.")

    # -------------------------
    # Joystick helpers
    # -------------------------

    def _apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def _joystick_axes_to_kv(self) -> dict:
        """Poll joystick axes (-255..255 each, incl. camera pan/tilt).
        Returns them only when they changed since the last poll, so a resting stick
        doesn't overwrite keyboard speed/steering every frame. Mapping is for a
        Kolke KGJ-182; adjust the JS_* constants for other pads.
        """

        js = self.joystick
        if js is None:
            return {}

        # axis indices should match your project constants (JS_AXIS_*)
        try:
            axis_omega = js.get_axis(JS_AXIS_RIGHT_HORIZONTAL)
            axis_speed = js.get_axis(JS_AXIS_RIGHT_VERTICAL)
            axis_pan = js.get_axis(JS_AXIS_LEFT_HORIZONTAL)
            axis_tilt = js.get_axis(JS_AXIS_LEFT_VERTICAL)
        except Exception:
            # joystick may not be ready or indices different
            return {}

        axis_omega = self._apply_deadzone(axis_omega)
        axis_speed = self._apply_deadzone(axis_speed)
        axis_pan = self._apply_deadzone(axis_pan)
        axis_tilt = self._apply_deadzone(axis_tilt)

        kv = {
            "speed_percent": int(-axis_speed * 255),    # forward/backward
            "omega_percent": int(axis_omega * 255),     # steering
            "cam_pan_percent": int(axis_pan * 255),  # camera pan (-255..255)
            # camera tilt (inverted)
            "cam_tilt_percent": int(-axis_tilt * 255),
        }
        if kv == self._last_axes_kv:
            return {}
        self._last_axes_kv = kv
        if self.debug:
            print("joystick_axes_to_kv:", kv)
        return kv

    def _joystick_event_to_kv(self, event) -> dict:
        """Handle joystick button events and return discrete inputs."""
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == JS_BTN_START:
                return {"toggle_pause": 1}
            elif event.button == JS_BTN_OPTIONS:
                return {"toggle_tracking": 1}
            elif event.button == JS_BTN_L1:
                return {"disturb_left": 1}
            elif event.button == JS_BTN_R1:
                return {"disturb_right": 1}
            elif event.button == JS_BTN_SQUARE:
                return {"horn_pressed": 1}

        elif event.type == pygame.JOYBUTTONUP:
            # Release events (optional, e.g. reset toggle or send 0)
            if event.button == JS_BTN_L1:
                return {"disturb_left": 0}
            elif event.button == JS_BTN_R1:
                return {"disturb_right": 0}
            elif event.button == JS_BTN_SQUARE:
                return {"horn_released": 1}

        return {}

    # -------------------------
    # Keyboard helpers
    # -------------------------
    def _keyboard_to_kv(self, key) -> dict:
        """Map pressed keys to simple key-value dict."""
        mapping = {
            pygame.K_a: {"turn_left": 1},
            pygame.K_d: {"turn_right": 1},
            pygame.K_z: {"disturb_left": 1},
            pygame.K_c: {"disturb_right": 1},
            pygame.K_w: {"faster": 1},
            pygame.K_s: {"slower": 1},
            pygame.K_p: {"toggle_pause": 1},
            pygame.K_t: {"toggle_tracking": 1},
        }
        return mapping.get(key, {})

    # -------------------------
    # High level poll
    # -------------------------
    def check_for_inputs(self) -> Tuple[Dict[str, Any], bool]:
        """
        Merge keyboard and joystick inputs into a single dict (joystick overrides keyboard).
        Returns (inputs, running); running is False once the window is closed.
        """
        inputs = {}
        running = True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                inputs.update(self._keyboard_to_kv(event.key))
            elif self.joystick and (event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP):
                inputs.update(self._joystick_event_to_kv(event))

        # Poll joystick separately each frame
        if self.joystick:
            inputs.update(self._joystick_axes_to_kv())

        return inputs, running
