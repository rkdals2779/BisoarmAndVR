"""
One Euro Filter
================
Casiez et al. 2012 1-euro filter.
적응형 저역통과 필터: 느릴 땐 강하게 스무딩, 빠를 땐 지연을 최소화.
numpy 벡터 전체에 대해 하나의 적응형 컷오프(속력 기반)를 공유해서 적용합니다.
위치(XYZ)뿐 아니라 임의 차원 벡터(예: [pitch, roll])에도 그대로 씁니다.
"""

import numpy as np


class OneEuroFilter:
	def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
		self.min_cutoff = min_cutoff
		self.beta = beta
		self.d_cutoff = d_cutoff
		self.x_prev = None
		self.dx_prev = None
		self.t_prev = None

	@staticmethod
	def _alpha(dt, cutoff):
		tau = 1.0 / (2 * np.pi * cutoff)
		return 1.0 / (1.0 + tau / dt)

	def filter(self, x, t):
		x = np.asarray(x, dtype=float)

		if self.t_prev is None:
			self.x_prev = x.copy()
			self.dx_prev = np.zeros_like(x)
			self.t_prev = t
			return x.copy()

		dt = max(t - self.t_prev, 1e-6)

		dx = (x - self.x_prev) / dt
		a_d = self._alpha(dt, self.d_cutoff)
		dx_hat = a_d * dx + (1 - a_d) * self.dx_prev

		speed = float(np.linalg.norm(dx_hat))
		cutoff = self.min_cutoff + self.beta * speed
		a = self._alpha(dt, cutoff)
		x_hat = a * x + (1 - a) * self.x_prev

		self.x_prev = x_hat
		self.dx_prev = dx_hat
		self.t_prev = t
		return x_hat.copy()
