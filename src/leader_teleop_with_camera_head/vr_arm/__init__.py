"""VR 컨트롤러 팔 텔레오퍼레이터 패키지.

모듈 구성:
	config.py - BiVrLeaderConfig (--teleop.type=bi_vr_leader 등록)
	teleop.py - BiVrLeader (openvr/scipy/ikpy 필요)

config는 가볍게 임포트되고(등록용), 무거운 의존성이 있는 teleop은
실제로 bi_vr_leader 타입이 선택됐을 때만 임포트하면 된다.
"""

from .config import BiVrLeaderConfig

__all__ = ['BiVrLeaderConfig']
