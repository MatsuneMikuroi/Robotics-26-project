import time as _time


class PID:

    TIME_STEP: int = 64

    def __init__(self, k: float, t_i: float, t_d: float) -> None:
        self.error: float = 0
        self.deriv: float = 0
        self.integ: float = 0
        self.K: float = k
        self.T_I: float = t_i
        self.T_D: float = t_d
        self._last_time: float = None

    def compute(self, prox: float, target: float) -> float:
        now = _time.monotonic()
        dt_ms = (now - self._last_time) * 1000 if self._last_time is not None else self.TIME_STEP
        self._last_time = now

        prev_err: float = self.error
        self.error = prox - target

        self.deriv = (self.error - prev_err) * 1000 / dt_ms
        self.integ += self.error * dt_ms / 1000

        return self.P() + self.I() + self.D()

    def P(self) -> float:
        return self.K * self.error

    def I(self) -> float:
        return self.K * (self.integ / self.T_I)

    def D(self) -> float:
        return self.K * (self.T_D * self.deriv)
