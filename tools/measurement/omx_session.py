#!/usr/bin/env python3
"""Single notebook inventory and recording archive. No camera/serial opens."""
import argparse
import json
import shutil
import tempfile
from pathlib import Path

from omx_arm_config import connected_serial_ports, load_arm_profile, ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('devices', 'archive', 'calibration-status'))
    args = parser.parse_args(argv)
    profile = load_arm_profile(None)
    if args.command == 'devices':
        connected = connected_serial_ports()
        print(json.dumps({'port': str(connected[0]) if connected else None, 'port_visible': bool(connected),
                          'camera': profile.camera, 'camera_visible': Path(profile.camera).exists(),
                          'width': profile.width, 'height': profile.height, 'fps': profile.fps,
                          'samples': str(profile.samples), 'calibration': str(profile.calibration),
                          'poses': str(profile.poses)}, ensure_ascii=False, indent=2))
        return 0
    if args.command == 'calibration-status':
        data = json.loads(profile.calibration.read_text())
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return 0
    files = [p for p in (profile.samples, profile.calibration, profile.poses) if p.exists()]
    if not files:
        print('[NEW_SESSION] 새 단일 구성의 기록이 아직 없습니다. 보관할 파일 없이 수집을 시작할 수 있습니다.')
        return 0
    print('새 배치로 다시 수집할 때만 사용합니다. 수집 재개라면 취소하세요.')
    for p in files:
        print(p)
    if input('현재 단일 구성의 보정·자세 기록을 보관하려면 ARCHIVE: ').strip() != 'ARCHIVE':
        print('[CANCELLED] 파일을 옮기지 않았습니다')
        return 0
    parent = ROOT / 'records/single/archive'
    parent.mkdir(parents=True, exist_ok=True)
    target = Path(tempfile.mkdtemp(prefix='previous_', dir=parent))
    for p in files:
        shutil.move(str(p), target/p.name)
    print('[ARCHIVED]', target)
    print('기존 다중 팔·구형 단일 팔 파일은 변경하지 않았습니다.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc))
