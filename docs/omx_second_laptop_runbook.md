# 두 번째 노트북: OMX YOLO 집기·적재 복사 실행 안내

작성: 2026-10-20. 노트북 1대 + OMX-F 1대 + 상단 카메라 1대 구성.
**이 문서의 명령을 코드 블록 하나씩 복사해서 실행한다.** 반복문과 경로 변수는 없다.
현재 노트북에서는 코드·모델 포장과 장비를 사용하지 않는 확인만 했다. 두 번째 노트북의 설치·연결·실물 동작은 그 노트북에서 확인한다.

## 1. Git 받기 또는 압축 해제와 환경 설치

GitHub에서 받는 방법을 기본으로 사용한다. 이미 `~/dev_ws/omxtest`가 있다면 덮어쓰지 않고 기존 폴더를 먼저 보관한다.
슬랙 ZIP으로 받은 경우에만 뒤의 ZIP 해제 명령을 대신 사용한다. Git과 ZIP을 같은 폴더에 겹쳐 받지 않는다.

```bash
cat /etc/os-release
python3 --version
```

현재 시험한 환경은 Ubuntu 24.04 계열의 Python 3.12.3이다. 아래 버전 고정 설치는 **Python 3.12 기준**이다. 다른 Python 버전이면 설치 전에 환경을 맞춘다.
비밀번호는 사용자가 터미널에서 직접 입력한다.

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y git python3-venv python3-pip unzip psmisc libgl1 libglib2.0-0 fonts-dejavu-core
```

아래는 새 폴더를 만드는 명령이다. 이미 Git으로 받았다면 이 두 명령은 생략한다.

```bash
mkdir -p ~/dev_ws
```

```bash
git clone https://github.com/rheffi/omxtest.git ~/dev_ws/omxtest
```


Git으로 받았다면 아래 ZIP 해제 명령 두 개는 건너뛴다. ZIP은 홈의 `Downloads` 폴더에 둔다.

```bash
mkdir -p ~/dev_ws
```

```bash
unzip -n ~/Downloads/omxtest_transfer_20261020.zip -d ~/dev_ws
```

```bash
cd ~/dev_ws/omxtest
```

```bash
env -u PYTHONPATH -u PYTHONHOME python3 -m venv .venv-pick
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -m pip install --upgrade pip
```

CPU용 PyTorch를 먼저 설치한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -m pip install -r requirements-transfer.txt
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -m pip check
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B -c 'import cv2, numpy, dynamixel_sdk, torch, ultralytics; print("OpenCV", cv2.__version__); print("NumPy", numpy.__version__); print("Torch", torch.__version__); print("Ultralytics", ultralytics.__version__)'
```

새 터미널을 열 때는 다음 명령으로 작업 폴더에 들어온 뒤 이어서 실행한다. 별도 `source`는 필요 없다.

```bash
cd ~/dev_ws/omxtest
```

## 2. 모터·카메라 확인

OMX-F 한 대와 상단 카메라 한 대를 연결한다. 현재 코드는 연결된 OpenRB와 **SNAP U2**를 찾는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/omx_session.py devices
```

`port_visible`, `camera_visible`이 `true`인지 확인한다. 카메라 모델이 SNAP U2와 다르거나 `false`이면 아래 목록으로 실제 장치부터 확인하고, 잘못된 카메라로 캘리를 시작하지 않는다.

```bash
ls -l /dev/serial/by-id/
```

```bash
ls -l /dev/v4l/by-id/
```

직렬 포트에 Permission denied가 나면 사용자 그룹을 확인한다.

```bash
id -nG
```

`dialout`이 없을 때만 아래 명령을 실행하고 로그아웃·로그인한다. 비밀번호는 직접 입력한다.

```bash
sudo usermod -aG dialout "$USER"
```

이 구성은 OpenRB 고유 장치를 자동 탐색하므로 `/dev/ttyACM0` 번호나 이전 노트북의 udev 별칭을 복사하지 않는다.
다른 제어 프로그램을 종료하고 팔을 받친 상태에서 모터를 읽는다. 이 명령은 목표·토크를 쓰지 않지만 포트를 열 때 보드가 재시작될 수 있다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --inspect
```

현재 실행 코드가 요구하는 모드는 팔 ID 11~15: 4, 그리퍼 ID 16: 5, Homing Offset: 0이다. 다르면 이 문서로 임의 변경하지 않고 기존 장비 설정을 확인한다.

카메라를 보며 선반·팔 베이스를 고정한다. `Q`/Esc로 종료한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/00_omx_camera_layout_check.py
```

나중에 위치를 비교할 이쪽 현장의 기준 사진은 아래 창에서 `S`로 저장한 뒤 `Q`로 닫는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/00_omx_camera_overlay.py
```

카메라가 흔들린 것 같을 때 같은 명령을 다시 실행하면 이 노트북에서 저장한 최신 사진을 현재 영상에 겹쳐 본다. 비교 창과 다른 카메라 창은 동시에 열지 않는다.

**Git 저장소에는 현장별 캘리·자세·사진을 포함하지 않는다. `config/single/`, `records/single/`는 수집 도구가 생성한다. ZIP 전달본에만 있는 `reference/site1/`도 참고용이며 다른 현장 기준 사진에 두 번째 카메라를 맞추지 않는다.**

## 3. 캘리 수집과 계산

팔을 받치고 아래 명령에서 `OFF`를 입력하면 수동 정렬할 수 있다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --torque-off
```

첫 수집이므로 이전 기록 이동·삭제 명령은 실행하지 않는다. 도중에 중단했다면 이미 수집한 다음 점부터 이어간다.


좌우 선반은 같은 작업 높이를 사용한다. 같은 십자 기준물을 쓰고 십자 윗면을 시험할 상품의 인식 기준면 높이에 맞춘다. 서로 다른 높이를 한 평면으로 간주하지 않는다.

첫 점에서 **선반 작업면부터 십자 윗면까지 실제 측정한 높이(mm)**를 입력하면 이후 점에서 재사용한다. 이전 값 20mm를 그대로 가정하지 않는다. 바구니 높이 차 110mm도 이 항목의 답이 아니다. 미측정이면 Enter로 기록할 수 있지만 높이 차이에 따른 정확도를 확인한 것은 아니다.

모든 점에서 다음 순서를 따른다.

1. 해당 위치에 십자 기준물을 놓고 명령을 실행한다.
2. 팔이 십자를 가리지 않은 영상에서 십자 중심 클릭 → Enter.
3. 기준물은 그대로 두고 그리퍼 중심을 수동으로 맞춘다. 같은 수직 방향·벌림 방향·십자와의 간격을 유지한다.
4. 터미널에 `READ` → 읽힌 값과 실제 정렬을 확인하고 `SAVE`.

`collect`는 한 점을 저장할 뿐 팔을 이동시키거나 토크를 켜지 않는다. 모서리는 실제 상품 배치 영역의 **화면상 좌상 → 우상 → 우하 → 좌하**다. 선반 사이 빈 공간에 점을 놓지 않는다.

### 왼쪽 좌상단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_1
```

### 왼쪽 우상단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_2
```

### 왼쪽 우하단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_3
```

### 왼쪽 좌하단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name left_corner_4
```

### 왼쪽 중앙

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name left_center
```

### 오른쪽 좌상단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_1
```

### 오른쪽 우상단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_2
```

### 오른쪽 우하단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_3
```

### 오른쪽 좌하단

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name right_corner_4
```

### 오른쪽 중앙

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --point-name right_center
```

### 독립 확인점 4개

앞의 10점과 다른 위치를 사용한다. 각 선반 영역 안에서 끝쪽과 내부를 하나씩 고른다. 같은 방식으로 클릭 → 수동 정렬 → `READ` → `SAVE`한다. 이 4점은 변환 행렬을 맞추는 데 넣지 않고 오차 확인에 사용한다.

### 왼쪽 끝쪽 확인점

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --purpose check --point-name left_check_edge
```

### 왼쪽 내부 확인점

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --purpose check --point-name left_check_inside
```

### 오른쪽 끝쪽 확인점

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --purpose check --point-name right_check_edge
```

### 오른쪽 내부 확인점

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py collect --purpose check --point-name right_check_inside
```

### 캘리 행렬 계산

14점 수집 후 실행한다. UV→모델 XY 행렬과 좌우 선반 영역을 저장한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py fit --layout two-shelves
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/omx_session.py calibration-status
```

`check_error.max_mm`와 `shelf_check_error.left/right`는 독립 확인점의 모델 XY 오차다. 실제 그리퍼 정확도와 동일한 측정은 아니며 이 숫자로 자동 차단하지 않는다. 같은 CSV로 의도적으로 재계산할 때만 다음 명령을 사용한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/05_omx_xy_calibration.py fit --layout two-shelves --overwrite
```

카메라·팔 베이스를 다시 움직였다면 이전 점에 새 점을 섞지 않고 새 수집을 시작한다.


## 4. HIGH·드랍 자세 기록


각 자세를 직접 만든 뒤 명령을 실행하고 `SAVE`를 입력한다. 명령 자체는 팔을 그 자세로 이동시키지 않는다. 이전 팔의 저장 펄스는 재사용하지 않는다.

### 높은 대기 자세

선반과 바구니 사이 이동에 사용할 높은 자세를 맞춘다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save hover_high
```

### 바구니 상부 접근 자세

바구니를 실제 지정 정차 위치에 두고 테두리 위의 접근 자세를 맞춘다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save drop_approach
```

### 바구니 놓기 자세

상품을 바구니 안으로 놓을 위치·높이에 맞춘다. 선반보다 110mm 아래라는 이유로 기존 모델 Z에서 110을 일괄 빼지 않고 실물 자세를 기록한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save drop_release
```

## 5. 호버·YOLO·집기 설정

### 클릭한 위치로 호버

기록한 `hover_high` 자세에서 시작한다. 카메라에서 십자 중심을 클릭하고 화면·터미널 안내에 따라 `MOVE`로 실행한다.
`--from-folded`는 붙이지 않는다. 기존 `06` 도구의 펼침 기본값을 두 번째 팔에 그대로 적용하지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/06_omx_xy_hover.py --live --stage-seconds 3
```

선반 여러 위치에서 실제 중심 정렬을 확인한다. 이후 수동으로 팔·그리퍼 상태를 맞출 때는 필요한 범위의 토크를 해제한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --torque-off
```

### YOLO 인식만 확인

모델에 학습된 상품을 선반에 놓는다. 모델은 저장소와 ZIP에 포함돼 있어 다시 학습하거나 별도로 다운로드할 필요가 없다. 이 명령은 모터에 연결하지 않는다.

```bash
cat weights/classes.txt
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --detect-only --show --weights weights/best.pt
```

### 상품·그리퍼 설정 기록

먼저 HIGH에서 출발하는 구성으로 설정한다. 모르는 숫자를 입력하는 방식이 아니라 손으로 맞춘 실제 상태를 읽는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick_setup.py --weights weights/best.pt
```

1. 상품 번호 선택.
2. 상품 중심을 실제로 잡을 높이·방향에 그리퍼를 맞춘 뒤 Enter: 집기 Z 기록.
3. 빈 그리퍼를 상품이 들어갈 만큼 벌리고 Enter: 열림값 기록.
4. 물건을 빼고 사용할 닫힘 상태까지 충분히 닫은 뒤 Enter: 닫힘값 기록. 덜 닫은 상태를 저장하지 않는다.
5. 실제 핑키프로 바구니는 `1`, 임시 바구니는 `2` 선택 → 위치 메모 → `SAVE`.

설정 도구 자체는 움직이지 않는다. 다른 제어 프로그램으로 자세를 맞췄다면 읽기 전에 그 프로그램을 종료한다.
수정본은 이때 기록한 **집게 방향도** 접근·호버·집기·상승에 반영한다. 기존 설정의 `setup_measurements.pick_arm_pulses`도 자동으로 읽으므로 방향 기록이 남아 있다면 다시 설정하지 않는다.
그리퍼 전류는 닫힘 기록에서 읽은 Goal Current를 재사용한다. 첫 번째 팔의 닫힘값 **2136**과 전류값을 그대로 복사하지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --status
```

### 빈 그리퍼 열림·닫힘 확인

각 명령에서 `GRIPPER`를 입력한다. 이 명령들은 그리퍼만 제어하며 종료 후 토크를 유지한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --gripper-test open --live
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --gripper-test close --live
```

그리퍼만 다시 손으로 조절하려면 아래 명령에서 `OFF`를 입력한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --torque-off --torque-scope gripper
```

## 6. YOLO 호버 → 집기·드랍

### 손목 회전 수정본: 먼저 계산 확인

카메라·모터를 열지 않고 기록한 집기점에서 경로를 계산한다. `wrist_path`의 `id15_unwrapped_pulse`와 `delta_deg`를 확인한다. 실제 YOLO 목표·도착·충돌을 검증하는 명령은 아니다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --wrist-preview
```

손목(ID 15)이 0/4095 경계를 넘을 때는 현재 위치와 가까운 동등 목표를 사용한다. 예를 들어 기록값 `26 → 4069`는 실제 전송 목표 `26 → -27`이 되어 약 4.7도만 움직인다. 음수 목표는 이미 요구하는 모드 4의 확장 위치값이며 모터 모드를 새로 바꾸지 않는다. 다른 팔 관절에 이 처리를 적용하지 않는다.

HIGH·드랍은 기록한 자세를 사용하므로 그 구간의 집게 방향은 기록에 따라 달라질 수 있다. 집기 구간의 방향 유지와 저장 자세 전환을 구분해서 확인한다. 자세를 다시 기록해야 한다면 실물 정렬 후 해당 자세만 교체하며 캘리 행렬을 다시 계산하지 않는다. 자세 재기록과 상세 설명은 `docs/omx_wrist_direction_fix.md`를 참고한다.

팔이 HIGH 자세인 상태에서 시작한다. 아래 호버는 상품 위로 이동했다가 HIGH로 복귀하며 그리퍼는 제어하지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --hover --live
```

호버가 맞으면 상품을 놓고 실행한다. `MOVE` 한 번으로 집기 → 들어올리기 → 바구니 드랍 → HIGH 복귀까지 진행한다. `GRIPPED` 입력은 없다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live
```

기본 시간은 일반 팔 이동 2.5초, 그리퍼 0.8초, 구간 사이 0.1초다. 임시 바구니와 실제 핑키프로 바구니 시험을 구분한다. 명령 종료와 실제 집기·적재 성공은 다르므로 물체가 유지되고 바구니 안에 들어갔는지 직접 확인한다.

### 중심이 어긋나면

HIGH에서 시작한다. 상품 위 이동 → 집기 직전 높이에서 확인 → `OFF` 후 손으로 실제 중심·집기 높이 정렬 → Enter → `SAVE` 순서다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick_adjust.py
```

선택 상품의 XY 오프셋과 집기 Z만 바꾸며 캘리 행렬은 바꾸지 않는다. 한 위치에서 맞춘 오프셋을 선반 전체가 맞는 것으로 간주하지 않는다.
5mm 위의 낮은 호버 위치를 그대로 집기 Z로 저장하지 않는다. 저장 후 자동 복귀는 없다.

### 토크 해제

집기 후 **초기 자세 복귀 → 그리퍼 닫힘 → 전체 토크 해제**까지 실행하려면
[복귀 자세 기록·시험 안내](omx_parking.md)에 따라 경유 자세와 초기 자세를 한 번 기록한 뒤 아래 명령을 사용한다.
2026-10-21 현재 노트북·장비에서 정상 동작을 확인했다. `MOVE`를 입력하면 전체 순서를 진행한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live --park-after --park-transit-tolerance 50
```

HIGH·중간 경유점의 허용 오차는 50펄스이며 최종 팔 자세와 그리퍼 닫힘 기준은 20펄스를 유지한다.
복귀만 실행하려면 `--park --live --park-transit-tolerance 50`을 사용한다.
그리퍼는 저장한 닫힘 펄스·전류 설정으로 닫은 뒤 토크를 해제한다.

팔·상품을 받친 상태에서 실행하고 `OFF`를 입력한다. 다음 수동 자세 조절 전에 사용한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --torque-off
```

## 선택: 접힌 상태에서 자동 출발하기

이 절은 두 번째 장비에서 펼침 순서를 직접 확인한 뒤 사용한다. 첫 테스트는 위의 HIGH 출발로 한다.
기존 설정을 펼침 기록까지 포함해 다시 저장하는 명령이다. 이미 기록한 상품·그리퍼 상태는 재사용하며 `--rerecord`는 붙이지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick_setup.py --weights weights/best.pt --from-folded --replace
```

펼침 기록에서 요구하는 것은 다음 두 상태다.

1. 실제 접힌 출발 상태에서 **ID 13 팔꿈치만** 먼저 펼친 중간 상태를 만들고 Enter.
2. 그 상태에서 **ID 12 어깨만** 움직인 다음 상태를 만들고 Enter.

실행 때 이 두 목표를 순서대로 적용한 뒤 저장된 HIGH로 이동한다. 기록 도구가 펼침을 대신 실행해 주지는 않는다.
선반과 손가락이 간섭하면 이 순서를 사용하지 말고 HIGH에서 출발한다. `[펼침 재사용]`이 나타나면 이전 하드코딩 중간값을 재사용하는 경우이므로 다른 팔에 맞는 것으로 취급하지 않는다.

접힌 상태에서 YOLO 호버:

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --hover --live --from-folded
```

접힌 상태에서 집기·드랍:

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live --from-folded
```

## 전달본 구성

- `tools/`: 손목 방향 유지·경계 통과 보상을 포함한 현재 실행 코드.
- `weights/best.pt`, `weights/classes.txt`: 기존 학습 결과와 클래스 목록.
- `requirements-transfer.txt`: 현재 성공한 주요 패키지 버전.
- `config/omx_pick.example.json`: 새 설정 생성용 예제.
- `config/single/`, `records/single/`: 이 노트북에서 새로 기록할 경로.
- `reference/site1/`: ZIP 전달본에만 포함된 첫 번째 현장 참고 자료. Git에는 포함하지 않는다.
- `MANIFEST.sha256`: ZIP 전달본 내부 파일 체크섬.

다른 노트북의 가상환경, 로그, 과거 백업은 포함하지 않았다. ROS 2나 LeRobot 설치는 이 Python 제어 경로의 실행 전제에 포함하지 않는다. 실제 모터 모드가 준비돼 있는지는 `--inspect`로 확인한다.
