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
        self._last_reset_time = 0.0

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
    _ENV_STEP_KEYS = ("phys", "render", "obs")

    def __init__(self):
        self.stats: dict[str, float] = {
            "advance": 0.0, "zmq_recv": 0.0, "send_status": 0.0, "send_camera": 0.0,
            "preprocess": 0.0, "env_step": 0.0, "total": 0.0, "count": 0,
            "phys": 0.0, "render": 0.0, "obs": 0.0,
        }
        self._last_print_time = time.time()

    def record(
        self,
        advance_ms: float,
        preprocess_ms: float,
        env_step_ms: float,
        advance_detail: dict | None = None,
        env_step_detail: dict | None = None,
    ):
        total_ms = advance_ms + preprocess_ms + env_step_ms
        self.stats["advance"] += advance_ms
        self.stats["preprocess"] += preprocess_ms
        self.stats["env_step"] += env_step_ms
        self.stats["total"] += total_ms
        if advance_detail is not None:
            for key in ("zmq_recv", "send_status", "send_camera"):
                self.stats[key] += advance_detail.get(key, 0.0)
        if env_step_detail is not None:
            for key in self._ENV_STEP_KEYS:
                self.stats[key] += env_step_detail.get(key, 0.0)
        self.stats["count"] += 1

    def maybe_print(self, interval: float = 2.0):
        now = time.time()
        if now - self._last_print_time < interval:
            return
        c = self.stats["count"]
        if c > 0:
            hz = c / (now - self._last_print_time)
            parts = [f"Hz: {hz:.2f}"]

            # Advance with sub-timings
            has_adv_sub = self.stats.get("zmq_recv", 0.0) > 0 or self.stats.get("send_camera", 0.0) > 0
            if has_adv_sub:
                parts.append(
                    f"Advance: {self.stats['advance']/c:.2f}ms "
                    f"(ZMQRecv: {self.stats['zmq_recv']/c:.2f}ms, "
                    f"SndStatus: {self.stats['send_status']/c:.2f}ms, "
                    f"SndCamera: {self.stats['send_camera']/c:.2f}ms)"
                )
            else:
                parts.append(f"Advance: {self.stats['advance']/c:.2f}ms")
            parts.append(f"Preprocess: {self.stats['preprocess']/c:.2f}ms")

            # EnvStep with sub-timings
            has_env_sub = self.stats.get("phys", 0.0) > 0 or self.stats.get("obs", 0.0) > 0
            phys_ms = self.stats["phys"] / c
            render_ms = self.stats["render"] / c
            obs_ms = self.stats["obs"] / c
            env_ms = self.stats["env_step"] / c
            other_ms = max(0, env_ms - phys_ms - render_ms - obs_ms)
            if has_env_sub:
                parts.append(
                    f"EnvStep: {env_ms:.2f}ms "
                    f"(Phys: {phys_ms:.2f}ms, "
                    f"Render: {render_ms:.2f}ms, "
                    f"Obs: {obs_ms:.2f}ms, "
                    f"Other: {other_ms:.2f}ms)"
                )
            else:
                parts.append(f"EnvStep: {env_ms:.2f}ms")
            parts.append(f"Total: {self.stats['total']/c:.2f}ms")
            print(f"[PERF] {' | '.join(parts)}", flush=True)

        self.stats = {
            "advance": 0.0, "zmq_recv": 0.0, "send_status": 0.0, "send_camera": 0.0,
            "preprocess": 0.0, "env_step": 0.0, "total": 0.0, "count": 0,
            "phys": 0.0, "render": 0.0, "obs": 0.0,
        }
        self._last_print_time = now
