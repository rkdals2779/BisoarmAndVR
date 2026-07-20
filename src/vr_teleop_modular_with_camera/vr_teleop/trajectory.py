"""
임계감쇠(critically damped) 관절 궤적 컨트롤러
================================================
~CONTROL_HZ로 들어오는 목표 관절각(deg) 스트림을, 속도/가속도가 제한된
물리적으로 매끄러운 움직임으로 변환합니다.

관절마다 임계감쇠(zeta=1) 스프링-댐퍼를 적용:
    accel = kp * (target - pos) - kd * vel,   kd = 2*sqrt(kp)

임계감쇠이므로 목표가 계속 움직여도 오버슈트/진동 없이 가속->감속
프로파일이 자연스럽게 나옵니다. wrist_flex/wrist_roll도 이 컨트롤러를
그대로 통과하므로, IK 없이 직접 매핑하더라도 갑자기 튀지 않고 부드럽게
움직입니다.
"""

import numpy as np


class JointTrajectoryController:
	def __init__(self, kp: float, max_vel_deg_s: float, max_acc_deg_s2: float):
		self.kp = kp
		self.kd = 2.0 * np.sqrt(kp)  # 임계감쇠 조건
		self.max_vel = max_vel_deg_s
		self.max_acc = max_acc_deg_s2
		self.pos = None
		self.vel = None

	def reset(self, pos_deg):
		self.pos = np.asarray(pos_deg, dtype=float).copy()
		self.vel = np.zeros_like(self.pos)

	def update(self, target_deg, dt):
		target_deg = np.asarray(target_deg, dtype=float)
		if self.pos is None:
			self.reset(target_deg)
			return self.pos.copy()

		error = target_deg - self.pos
		accel = self.kp * error - self.kd * self.vel
		accel = np.clip(accel, -self.max_acc, self.max_acc)

		self.vel = self.vel + accel * dt
		self.vel = np.clip(self.vel, -self.max_vel, self.max_vel)

		self.pos = self.pos + self.vel * dt
		return self.pos.copy()
