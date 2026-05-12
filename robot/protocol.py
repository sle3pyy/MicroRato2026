from __future__ import annotations

import sys
from pathlib import Path


def _ensure_pclient_on_path():
    project_root = Path(__file__).resolve().parents[2]
    pclient_dir = project_root / "ciberRatoTools" / "pClient"
    pclient_dir_str = str(pclient_dir)
    if pclient_dir_str not in sys.path:
        sys.path.insert(0, pclient_dir_str)


_ensure_pclient_on_path()

from croblink import CRobLink, CRobLinkAngs, CMeasures, NUM_IR_SENSORS, NUM_LINE_ELEMENTS  # noqa: E402

RobLink = CRobLink
RobLinkAngs = CRobLinkAngs
Measures = CMeasures

__all__ = [
    "RobLink",
    "RobLinkAngs",
    "Measures",
    "CRobLink",
    "CRobLinkAngs",
    "CMeasures",
    "NUM_IR_SENSORS",
    "NUM_LINE_ELEMENTS",
]
