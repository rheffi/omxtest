# OMX-F YOLO 집기·바구니 적재

노트북 한 대 + OMX-F 한 대 + 상단 SNAP U2 한 대 구성으로, YOLO 상품 인식 결과를 로봇 좌표로 변환해 집고 지정 바구니에 적재한다.
모방학습이나 VLA를 사용하는 실행 경로는 아니다.

## 다른 노트북에서 시작

```bash
mkdir -p ~/dev_ws
```

```bash
git clone https://github.com/rheffi/omxtest.git ~/dev_ws/omxtest
```

```bash
cd ~/dev_ws/omxtest
```

[설치 → 장비 확인 → 캘리 → 자세·그리퍼 기록 → 호버 → 집기·드랍 명령 안내](docs/omx_second_laptop_runbook.md)를 순서대로 실행한다.
`git clone`으로 받았다면 안내 1절의 ZIP 해제 대신 Git 경로를 사용한다.

## 포함된 파일

| 항목 | 위치 |
|---|---|
| 카메라 확인·기준 사진 비교 | `tools/measurement/00_omx_camera*.py` |
| UV → 로봇 XY 캘리 | `tools/measurement/05_omx_xy_calibration.py` |
| 클릭 호버 | `tools/measurement/06_omx_xy_hover.py` |
| HIGH·바구니 자세 기록 | `tools/measurement/07_omx_pose_recorder.py` |
| 상품·그리퍼 상태 기록 | `tools/picking/omx_pick_setup.py` |
| YOLO 인식·집기·드랍·토크 해제 | `tools/picking/omx_pick.py` |
| 집기점 오프셋·높이 조절 | `tools/picking/omx_pick_adjust.py` |
| 학습 모델·클래스 목록 | `weights/best.pt`, `weights/classes.txt` |
| 시험 환경의 주요 패키지 버전 | `requirements-transfer.txt` |

현재 전달 환경은 Ubuntu 24.04 / Python 3.12 / CPU PyTorch 2.7.1이다.
모델은 포함돼 있으며, 가상환경은 각 노트북에서 새로 설치한다.
이 Python 제어 경로는 실행 전제로 ROS 2나 LeRobot을 요구하지 않는다.

## 장비마다 새로 만드는 값

캘리·현재 장치 정보·저장 자세·집기 설정은 로컬 파일로 남기며 Git으로 공유하지 않는다.

- `config/single/`: 카메라–로봇 캘리와 저장 자세·측정 캐시
- `records/single/`: 수집한 대응점
- `config/omx_pick.json`: 해당 팔의 집기 Z·그리퍼 값·바구니 설정

좌우 선반은 같은 기준면 높이에서 보정한다. 바구니는 고정 정차 위치에서 실제 접근·놓기 자세를 기록한다.
다른 팔의 펄스·캘리·닫힘값을 복사하지 않는다. 실제 집기·적재 여부는 물체를 보고 확인한다.

## 코드만 업데이트

수집을 마친 파일은 Git 추적 대상이 아니므로 그대로 유지된다. 로컬 코드 수정이 있다면 먼저 확인한다.

```bash
cd ~/dev_ws/omxtest
```

```bash
git status --short
```

```bash
git pull --ff-only
```

## 장비 없이 실행 순서 확인

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --demo
```

## 초기 자세 복귀·토크 해제

[초기 자세와 경유 자세를 기록하고 빈 그리퍼로 복귀 시험](docs/omx_parking.md)한 뒤 아래 명령을 실행한다.
2026-10-21 현재 노트북·장비에서 정상 동작을 확인한 명령이다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live --park-after --park-transit-tolerance 50
```

집기 → 바구니 드랍 → HIGH → 초기 자세 복귀 → 그리퍼 닫힘 확인 → 팔·그리퍼 토크 OFF 순서다.
`--park-transit-tolerance 50`은 HIGH·중간 경유점에 적용되며 최종 팔 자세와 그리퍼 닫힘 기준은 20펄스다.
동일한 중간 허용 오차로 복귀만 실행하려면 `--park --live --park-transit-tolerance 50`을 사용한다.
현재 코드는 기록한 집기 방향 유지와 손목 0/4095 경계의 짧은 회전도 반영한다. [손목 방향 수정 안내](docs/omx_wrist_direction_fix.md)를 참고한다.

## 오프라인 회귀 테스트

가상 모터와 `tests/fixtures/`의 가상 데이터로 손목 경로·초기 자세 복귀·그리퍼 닫힘·토크 해제 순서를 검사한다. 현장 캘리브레이션·카메라·모터 연결은 필요 없다. 테스트 데이터는 실물 제어에 사용하지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B -m unittest discover -s tests -v
```
