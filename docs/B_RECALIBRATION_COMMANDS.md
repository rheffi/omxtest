# B OMX 재캘리·집기·3슬롯 적재 명령서

2026-10-29. **B 노트북의 그래픽 데스크톱 터미널**에서 실행한다.
아래 팔 동작·토크 해제 명령은 준비 과정에서 실행하지 않았다.

## 현재 결과와 다음 할 일

- [x] B 좌우 재캘리, 선반 공통 XY 및 라면 선반별 추가 XY·절대 집기 Z 반영.
- [x] slot0/1/2_release 재기록. 선반·슬롯 접근 자세는 유지.
- [x] 사용자 현장 보고: **3회 적재 적절히 성공**. 세 번째 상품(새우깡/치토스)과 단일 연속 실행 여부는 미확정. 신라면 실물 성공·새우깡 검출 해결로 확대 해석하지 않음.
- [x] B 직접 실행과 주문 worker의 pick_config를 최신 시험 설정으로 연결.
- [ ] Pinky 주행 → B 정밀주차 → 실제 적재를 관제에서 연결 시험.
- [ ] 주문 기반 시험 시 서버·필요 worker 기동. C 하역은 준비 완료까지 보류.

**아래 2~4절은 재작업이 필요할 때만 참고한다. 현재 새 캘리와 보정을 유지하며 ‘새 측정 시작·기존 XY 제거’를 다시 실행하지 않는다.**
현재 모터 USB는 연결 확인했다. 실행 당일 점유·연결 상태는 달라질 수 있다.

## 1. 터미널 준비·연결 확인

```bash
cd /home/jimmy/dev_ws/omxtest
py() { env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B "$@"; }
ls -l /dev/serial/by-id/
py tools/picking/omx_pick.py --config config/omx_pick_xy_trial.json --status
```

`py`는 현재 터미널에서만 유효하다. 새 터미널에서는 위 정의를 다시 실행한다.
시험 설정은 **B의 기존 설정으로 만든** `config/omx_pick_xy_trial.json`이다.
팔 이동·펼침은 6초, 그리퍼 시간은 기존 B 값을 유지한다.
관제 `gateway-b.hardware.json`과 주문용 `pinky-control-gateway-b.json` 모두 이 시험 설정을 읽도록 연결했다. 기존 `config/omx_pick.json`은 보존했으며 최신 시험과 보정값이 다르다.
자세·캘리는 `config/single/omx_saved_poses.json`, `config/single/omx_xy_calibration.json`을 사용한다.
A의 14번 +1024 보정 파일·자세·USB 설정은 B에 복사하지 않는다.

## 2. 새 캘리 측정 시작 — 완료, 현재 재실행하지 않음

기존 측정점과 새 카메라 구도의 측정점을 섞지 않는다. 이 명령은 기존 자료를 보관하고 새 측정을 시작한다. 이미 새 측정을 시작했다면 반복하지 않는다.

```bash
py - <<'PY'
from pathlib import Path
from datetime import datetime
import shutil
root = Path('records/single/archive') / ('b_recal_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
root.mkdir(parents=True)
for name in ('config/omx_pick_xy_trial.json', 'config/single/omx_xy_calibration.json', 'config/single/omx_saved_poses.json'):
    p = Path(name)
    if p.exists(): shutil.copy2(p, root / p.name)
samples = Path('records/single/omx_xy_samples.csv')
if samples.exists(): shutil.move(str(samples), str(root / samples.name))
print('백업:', root)
PY
```

손으로 팔 자세를 잡으려면 **팔을 받친 상태에서** 아래 토크 해제를 실행한다.

```bash
py tools/picking/omx_pose_move.py --torque-off
```

각 점에서 화면의 십자와 실제 그리퍼 중심을 맞춘다. 프로그램의 `READ`, `SAVE` 입력 안내를 따른다. 카메라·상품/측정점 배치를 중간에 바꾸지 않는다.

```bash
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_1
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_2
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_3
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_4
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_center
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_1
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_2
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_3
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_4
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_center
py tools/measurement/05_omx_xy_calibration.py collect --point-name left_check --purpose check
py tools/measurement/05_omx_xy_calibration.py collect --point-name right_check --purpose check
py tools/measurement/05_omx_xy_calibration.py fit --layout two-shelves --overwrite
```

새 캘리 저장에 성공한 뒤 아래를 **한 번만** 실행한다. 시험 설정의 기존 XY 추가 보정만 제거해 새 캘리에 중복 적용되지 않게 한다. 상품 집기 Z·그리퍼·자세는 유지한다. 이후 새로 맞춘 보정까지 지우지 않도록 반복하지 않는다.

```bash
py - <<'PY'
from pathlib import Path
import json
p = Path('config/omx_pick_xy_trial.json')
d = json.loads(p.read_text())
for adjustment in d.get('shelf_adjustments', {}).values():
    xyz = adjustment.get('delta_xyz_mm', [0, 0, 0])
    adjustment['delta_xyz_mm'] = [0, 0, xyz[2]]
for rule in d['classes'].values():
    rule['xy_offset_mm'] = [0, 0]
    for adjustment in rule.get('pick_adjustments_by_shelf', {}).values():
        adjustment.pop('xy_offset_mm', None)
p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + '\n')
print('B 시험 설정: 기존 XY 추가 보정 제거. Z·그리퍼 유지.')
PY
```

## 3. 낮은 집기 위치 확인·키보드 기록

먼저 검출만 확인한다. 모터 동작 없음.

```bash
py tools/picking/omx_pick.py --config config/omx_pick_xy_trial.json --detect-only --show
```

아래는 **실제 팔 이동**이다. 열린 그리퍼로 집기 위치 5mm 위까지 이동해 유지한다. 팔이 접혀 있을 때만 `--from-folded`를 붙인다.

```bash
py tools/picking/omx_pick.py --config config/omx_pick_xy_trial.json --class jin_ramen --hover --hover-mm 5 --stage-seconds 6 --live
```

오차를 직접 맞춰 기록하려면 상품을 같은 자리에 둔 채 아래 명령을 실행한다. 왼쪽·오른쪽은 해당 상품이 놓인 선반에 맞춰 사용한다.

```bash
py tools/picking/omx_pick_adjust.py --config config/omx_pick_xy_trial.json --class jin_ramen --shelf left --xy-record --point-name left_jin --hover-mm 5 --stage-seconds 6
py tools/picking/omx_pick_adjust.py --config config/omx_pick_xy_trial.json --class rice_chip --shelf left --xy-record --point-name left_rice --hover-mm 5 --stage-seconds 6
py tools/picking/omx_pick_adjust.py --config config/omx_pick_xy_trial.json --class shin_ramen --shelf right --xy-record --point-name right_shin --hover-mm 5 --stage-seconds 6
```

조절창에서 WASD: XY, R/F: Z, Enter: 기록·종료. `records/single/omx_xy_alignment_samples.jsonl`에 저장하며 **설정에는 자동 반영하지 않는다**. 기록을 전달하면 B의 측정값을 기준으로 보정한다. 임시 회피용 Z 이동과 실제 집기 높이 맞춤을 구분해 전달한다.

## 4. 필요한 자세만 재기록

저장 목록 조회:

```bash
py tools/picking/omx_pose_move.py --list
```

손으로 조정할 때 팔을 받친 상태에서 토크를 해제한다. 원하는 자세로 맞춘 뒤 해당 저장 명령 하나를 실행하고 안내에 따라 저장한다. 슬롯 번호는 코드 기준 0·1·2다.

```bash
py tools/picking/omx_pose_move.py --torque-off
py tools/measurement/07_omx_pose_recorder.py --save hover_high --replace
py tools/measurement/07_omx_pose_recorder.py --save shelf_left_approach --replace
py tools/measurement/07_omx_pose_recorder.py --save shelf_right_approach --replace
py tools/measurement/07_omx_pose_recorder.py --save slot0_approach --replace
py tools/measurement/07_omx_pose_recorder.py --save slot0_release --replace
py tools/measurement/07_omx_pose_recorder.py --save slot1_approach --replace
py tools/measurement/07_omx_pose_recorder.py --save slot1_release --replace
py tools/measurement/07_omx_pose_recorder.py --save slot2_approach --replace
py tools/measurement/07_omx_pose_recorder.py --save slot2_release --replace
```

모두 다시 기록하라는 뜻은 아니다. 현재 문제가 있는 자세만 선택한다. 손목 정책은 fixed를 유지한다.

## 5. 3개 연속 적재 — 상품 선택 가능

Pinky 바구니를 정밀주차 위치에 두고 실행한다. 아래는 **실제 집기·적재 명령**이다.

```bash
py tools/picking/omx_pick_three.py --config config/omx_pick_xy_trial.json --stage-seconds 6 --live
```

팔이 접힌 상태로 시작할 때만:

```bash
py tools/picking/omx_pick_three.py --config config/omx_pick_xy_trial.json --from-folded --stage-seconds 6 --live
```

옵션 생략 시 진라면→slot0, 라이스칩→slot1, 신라면→slot2. 상품마다 재인식하며 마지막 HIGH 복귀까지 목표를 전송한다. 출력의 SENT/END와 실제 성공은 구분한다.

현장에서 사용한 상품 선택 명령:

```bash
py tools/picking/omx_pick_three.py --config config/omx_pick_xy_trial.json --classes jin_ramen rice_chip shrimp_cracker --stage-seconds 6 --live
```

세 번째를 치토스로 바꾸려면 `--classes jin_ramen rice_chip cheetos`를 사용한다. 새우깡 검출 불안정이 해결됐다고 확정하지 않았다.

## 6. 관제로 넘길 결과

- [ ] 사용한 설정·자세·캘리 파일 경로와 마지막 수정 시각.
- [ ] 각 상품의 실제 집기·슬롯 적재 성공/실패 및 남은 문제.
- [ ] 기록만 저장한 XY 조정값의 반영 여부.
- [x] 관제에 최종 시험 설정·6초 팔 이동 연결. 실제 worker 기동·실물 연동 시험은 별도.
- [ ] 관제를 통한 전체 시나리오 시험.

로컬 실물 성공 보고와 관제 전체 시험 완료는 구분한다. C 하역은 아직 보류다.
