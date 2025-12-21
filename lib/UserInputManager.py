import pygame
from typing import Optional, Dict, Any

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
    """
    Shadow implementation that centralizes keyboard, joystick and external-comm
    inputs. Keep existing standalone functions (keyboard_to_kv, joystick_axes_to_kv)
    untouched — this class is additive so you can enable it incrementally.
    """

    def __init__(self,
                 comm_module: Optional[Any] = None,
                 deadzone: float = 0.2,
                 debug: bool = False):
       # self.comm = comm_module
        self.deadzone = deadzone
        self.debug = debug

        # ----------------------------------------------------
        # Try initialize joystick

        pygame.joystick.init()
        self.joystick = None

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
        """Poll joystick axes every frame and return continuous inputs.
            Map joystick buttons/axes to ControlInput.
            Adjust mapping for Kolke KGJ-182 if needed.
            Return raw joystick state as key-value dict.
            Includes camera pan/tilt axes (mapped to percent).
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
    # Comm/external helpers
    # -------------------------
    # def comm_to_kv(self) -> Dict[str, Any]:
    #     """Poll provided comm module for key-value inputs. Supports comm.read_kv() or comm.poll()."""
    #     if self.comm is None:
    #         return {}
    #     # try common method names without raising
    #     if hasattr(self.comm, "read_kv"):
    #         try:
    #             return self.comm.read_kv() or {}
    #         except Exception:
    #             return {}
    #     if hasattr(self.comm, "poll"):
    #         try:
    #             return self.comm.poll() or {}
    #         except Exception:
    #             return {}
    #     return {}

    # -------------------------
    # High level poll
    # -------------------------
    def check_for_inputs(self) -> Dict[str, Any]:
        """
        Merge keyboard, joystick and comm inputs into a single dict.
        Order: comm <- keyboard <- joystick (joystick overrides keyboard, keyboard overrides comm).
        """
        # inputs: Dict[str, Any] = {}
        # # merged.update(self.comm_to_kv())
        # inputs.update(self._keyboard_to_kv())
        # inputs.update(self._joystick_axes_to_kv())
        # if self.debug and inputs:
        #     print("check_for_inputs run successfully with output:", inputs)
        # return inputs

        # Check for inputs: local (keyboard or joystick + remote)

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

        # -------------------------------------------------------------------
        # Handle incoming data from surrogate robot (remote inputs)
        # remote_inputs = comm.receive()
        # if remote_inputs:
        #     print("Remote inputs:", remote_inputs)

    # -------------------------
    # Utilities
    # -------------------------

    def set_joystick(self, joystick: pygame.joystick.Joystick):
        self.joystick = joystick

    def set_comm(self, comm_module: Any):
        self.comm = comm_module

    def set_deadzone(self, deadzone: float):
        self.deadzone = deadzone

    def set_debug(self, enabled: bool):
        self.debug = enabled
