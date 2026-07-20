"""VR 컨트롤러 팔 텔레오퍼레이터 설정.

`@TeleoperatorConfig.register_subclass('bi_vr_leader')`로 등록되므로,
이 모듈이 임포트된 뒤에는 CLI에서 `--teleop.type=bi_vr_leader`로 선택할
수 있다 (lerobot 소스 수정 없음).
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass('bi_vr_leader')
@dataclass
class BiVrLeaderConfig(TeleoperatorConfig):
	"""VR 컨트롤러(좌/우)로 팔로워암을 조종하는 텔레옵 설정.

	세부 튜닝(필터/IK/손목 매핑/궤적)은 vr_teleop 패키지의 프리셋
	(LEFT_ARM_CONFIG / RIGHT_ARM_CONFIG)을 그대로 사용하고, 여기서는
	자주 바꾸는 값만 노출한다.

	Attributes:
		mode: 조종할 팔 ('left'/'right'/'dual').
		urdf_path: IK용 URDF 경로 (None이면 레포 기본 URDF).
		scale_factor: VR 이동량 -> 로봇 이동량 배율.
		gripper_open_pos: 트리거를 뗐을 때 그리퍼 값.
		gripper_close_pos: 트리거를 끝까지 당겼을 때 그리퍼 값.
	"""

	mode: str = 'dual'
	urdf_path: str | None = None
	scale_factor: float = 0.8
	gripper_open_pos: float = 90.0
	gripper_close_pos: float = 0.0

	def __post_init__(self) -> None:
		"""설정값 유효성 검증.

		Raises:
			ValueError: mode가 left/right/dual이 아닌 경우.
		"""
		if self.mode not in ('left', 'right', 'dual'):
			raise ValueError(
				f'mode는 left/right/dual 중 하나여야 합니다: {self.mode!r}'
			)
