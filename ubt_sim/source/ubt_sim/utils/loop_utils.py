"""Utilities for simulation main loops: rate limiting, keyboard reset, performance monitoring."""

import time


class KeyboardResetController:
    def __init__(self):
        import carb
        import omni

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard() if self._appwindow else None
        self._keyboard_sub = None
        if self._keyboard is not None:
            self._keyboard_sub = self._input.subscribe_to_keyboard_events(
                self._keyboard,
                self._on_keyboard_event,
            )
        self.reset_requested = False
        self.episode_complete_requested = False
        self._last_reset_time = 0.0
        self._last_episode_complete_time = 0.0

    def __del__(self):
        if self._keyboard is not None and self._keyboard_sub is not None:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)

    def _on_keyboard_event(self, event, *args, **kwargs):
        import carb

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            current_time = time.time()
            if event.input.name == "R":
                if current_time - self._last_reset_time > 1.0:
                    self.reset_requested = True
                    self._last_reset_time = current_time
            elif event.input.name == "SPACE":
                if current_time - self._last_episode_complete_time > 1.0:
                    self.episode_complete_requested = True
                    self._last_episode_complete_time = current_time
        return True


class RateLimiter:
    def __init__(self, hz):
        self.hz = hz
        self.last_time = time.perf_counter()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.0166, self.sleep_duration)

    def update_from_env(self, env):
        try:
            sim_dt = float(env.cfg.sim.dt)
            dec = int(getattr(env.cfg, "decimation", 1))
            env_sleep = sim_dt * max(1, dec)
            self.sleep_duration = max(env_sleep, 1.0 / self.hz)
            self.render_period = min(0.0166, self.sleep_duration)
        except Exception:
            self.sleep_duration = 1.0 / self.hz
            self.render_period = min(0.0166, self.sleep_duration)

    def sleep(self, env):
        """Attempt to sleep at the specified rate in hz."""
        now = time.perf_counter()
        elapsed = now - self.last_time
        to_sleep = self.sleep_duration - elapsed
        if to_sleep > 0:
            time.sleep(to_sleep)
            self.last_time += self.sleep_duration
        else:
            self.last_time = now


class PerfMonitor:
    def __init__(self):
        self.stats = {"advance": 0.0, "preprocess": 0.0, "env_step": 0.0, "total": 0.0, "count": 0}
        self._last_print_time = time.time()

    def record(self, advance_ms: float, preprocess_ms: float, env_step_ms: float):
        total_ms = advance_ms + preprocess_ms + env_step_ms
        self.stats["advance"] += advance_ms
        self.stats["preprocess"] += preprocess_ms
        self.stats["env_step"] += env_step_ms
        self.stats["total"] += total_ms
        self.stats["count"] += 1

    def maybe_print(self, interval: float = 2.0):
        now = time.time()
        if now - self._last_print_time < interval:
            return
        c = self.stats["count"]
        if c > 0:
            print(
                f"[PERF] Hz: {c/(now - self._last_print_time):.2f} | "
                f"Advance: {self.stats['advance']/c:.2f}ms | "
                f"Preprocess: {self.stats['preprocess']/c:.2f}ms | "
                f"EnvStep: {self.stats['env_step']/c:.2f}ms | "
                f"Total: {self.stats['total']/c:.2f}ms",
                flush=True,
            )
        self.stats = {"advance": 0.0, "preprocess": 0.0, "env_step": 0.0, "total": 0.0, "count": 0}
        self._last_print_time = now
