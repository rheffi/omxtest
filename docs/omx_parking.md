# 집기 후 초기 자세 복귀·토크 해제

수정한 `tools/picking/omx_pick.py`는 `--park-after`로 집기/호버 완료 후
HIGH → 기록한 경유 자세 → 기록한 초기 자세 → 그리퍼 닫힘 → 팔·그리퍼 토크 OFF를 실행한다.
기존 `--live`만 사용하면 HIGH에서 토크를 유지하는 동작은 그대로다.

초기 자세는 **토크가 없어도 받침이나 작업대가 팔을 지지하는 자세**여야 한다.
프로그램은 관절 위치와 정지를 읽을 수 있지만 받침 접촉이나 충돌 여부는 감지하지 못한다.
다른 장비의 펄스, 펼침의 역순, 임의의 접힘 각도를 기본값으로 사용하지 않는다.

## 1. 처음 한 번: 실제 장비에서 두 자세 기록

아래 명령은 `omxtest` 폴더에서 실행한다. 팔을 받치고 기존 토크 해제 명령을 실행해 `OFF`를 입력한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --torque-off
```

빈 그리퍼로 HIGH에서 초기 자세까지 내려가는 경로를 손으로 확인한다.
먼저 중간 경유 자세를 만들고 아래 명령을 실행해 `SAVE`를 입력한다.
HIGH에서 이 자세까지 모든 관절이 함께 보간되어 움직일 때도 간섭이 없어야 한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save park_approach
```

다음으로 팔을 받침 위 초기 자세에 놓고 아래 명령을 실행해 `SAVE`를 입력한다.
경유 자세에서 이 자세까지의 보간 경로도 확인한다. 기록 명령은 모터를 움직이지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save park_rest
```

이미 같은 이름을 저장했다면 해당 명령에 `--replace`를 붙여 `REPLACE`로 교체한다.
실행용 자세는 장비 식별 정보와 함께 `config/single/omx_saved_poses.json`에 저장된다.
이 전달본에는 실제 장비의 초기 자세가 포함되어 있지 않으므로 현장에서 기록해야 한다.

## 2. 복귀 경로만 먼저 시험

이 명령은 기록 내용을 출력할 뿐 카메라나 모터 포트를 열지 않는다.
캘리브레이션이나 검출 모델은 필요 없지만, `--config`의 그리퍼 닫힘 펄스·전류 설정은 필요하다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --park
```

팔을 손으로 HIGH에 두고 실제 경로와 마지막 받침을 확인한 뒤 실행한다.
`PARK`를 입력하면 **현재 위치에서** 경유 자세, 초기 자세 순서로 이동한다.
각 구간은 기본 4초다. 마지막 팔 자세 도착·정지를 확인한 뒤 저장된 그리퍼 닫힘 펄스로 닫는다.
닫힘·정지와 최종 팔 자세를 다시 확인한 다음 팔과 그리퍼의 토크를 모두 해제한다.
빈 그리퍼로 시험하고 팔을 받칠 준비를 한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --park --live
```

경유점이 더 필요하면 기록 도구로 각각 저장하고 전체 순서를 반복 지정한다.
최소 2개의 자세가 필요하며 **마지막 이름이 받침 위 초기 자세**다.
아래 사용자 지정 순서는 이후 `--park-after`에도 똑같이 지정해야 한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --park --live --park-pose park_approach --park-pose park_via2 --park-pose park_rest --park-seconds 5
```

## 3. 집기 후 자동 복귀

아래 명령은 2026-10-21 현재 노트북·장비에서 집기·드랍 후 초기 자세 복귀, 그리퍼 닫힘, 토크 해제까지 정상 동작을 확인했다.
`--park-transit-tolerance 50`을 함께 지정한다. 다른 장비에서는 해당 장비의 경로와 도착 오차를 확인한다.

복귀 시험이 끝났으면 팔을 다시 HIGH에 두고 실행한다.
`MOVE` 입력 전에 출력되는 복귀 순서를 확인한다.
한 번의 `MOVE`로 집기 → 드랍 → HIGH 도착 확인 → 초기 자세 복귀 → 그리퍼 닫힘 → 토크 OFF까지 진행한다.
추가 `OFF` 입력은 없으며 최종 자세에서 팔을 지지할 받침을 미리 준비한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live --park-after --park-transit-tolerance 50
```

이미 접힌 상태의 펼침 경로를 기록·시험했다면 `--from-folded`를 함께 사용할 수 있다.
복귀 완료 후 다음 실행도 접힌 상태에서 시작하려면 이 옵션이 필요하다.
`--hover --live --park-after`는 호버 중에는 그리퍼를 움직이지 않고,
초기 자세에 복귀한 뒤 그리퍼를 닫는다. 복귀 단독 명령인 `--park --live`도 동일하다.
복귀 중에는 그리퍼가 비어 있어야 하고, 초기 자세에서 닫힐 때도 손가락·작업대와 간섭이 없어야 한다.
닫힘 목표는 `config/omx_pick.json`의 `gripper.close_pulse`, 전류는 `gripper.current_limit_raw`를 사용한다.
닫힘 이동 시간은 `gripper_seconds`(없으면 `stage_seconds`)를 사용한다.
추가 옵션 없이 적용되며, 토크 해제 후에는 외력으로 그리퍼가 열릴 수 있다.

복귀 시간은 `--park-seconds 5`처럼 변경한다(1~60초).
`--stage-seconds`는 집기 동작 시간이고 복귀 시간은 별도다.

## HIGH 도착 오차 때문에 복귀 시작 전에 멈춘 경우

`RETURN_HIGH` 다음에 도착 확인 오류가 났다면, 아직 `park_approach` 이동은 시작하지 않은 상태다.
집기를 다시 실행할 필요 없이 현재 위치에서 `--park --live`로 복귀만 실행할 수 있다.
현재 위치부터 저장한 경유점까지의 경로와 그리퍼가 비어 있는지 먼저 확인한다.

기본 허용 오차는 20펄스다. 예를 들어 HIGH의 목표 대비 오차가 ID 12에서 +33,
ID 14에서 +42펄스로 남으면 기본 기준을 통과하지 못한다.
이 로그만으로 오차의 원인이 하중·제어 특성인지, 실제 간섭인지는 구분할 수 없다.
팔이 걸리거나 받침에 너무 일찍 닿아 멈춘 상태라면 허용 오차를 늘리지 않는다.

실제 경로에 간섭이 없음을 확인한 뒤에는 **HIGH와 중간 경유점만** 허용 오차를 명시적으로 지정할 수 있다.
아래 50펄스는 위 관측 오차를 수용하기 위한 선택값이며 장비의 물리적 안전을 보증하는 수치는 아니다.
마지막 초기 자세의 허용 오차는 이 옵션과 무관하게 20펄스를 유지한다.

현재 위치에서 복귀만 실행하고 `PARK`를 입력한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --park --live --park-transit-tolerance 50
```

다음 집기부터 같은 조건을 사용하려면 실행할 때 함께 지정한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live --park-after --park-transit-tolerance 50
```

`[PARK ARRIVED]`에 자세 이름과 최대 오차·허용 오차가 출력된다.
`park_rest`에서 다시 멈추면 최종 자세에 도착하지 않은 것이므로 강제로 토크를 끄지 않는다.
오류의 관절별 오차를 확인하고, 받침 위치와 실제 정렬을 점검한 뒤 필요하면 `park_rest`를 다시 기록한다.
급히 수동으로 복귀해야 한다면 팔을 받친 상태에서 기존 `--torque-off` 명령으로 해제하고 손으로 초기 자세에 둔다.

## 오류 처리와 확인 범위

- 모든 복귀 자세를 카메라·모터 연결 전에 읽는다. 기록 누락이나 장비 식별 불일치 시 시작하지 않는다.
- HIGH·중간 경유 자세의 허용 오차는 기본 20펄스이며 `--park-transit-tolerance`로 1~100펄스 범위에서 지정한다. 최종 초기 자세의 기준은 항상 20펄스다. 모든 관절에서 연속 측정 변화가 2펄스 이하로 0.5초 유지되는지도 확인한다.
- 구간 목표 전송 후 10초 내 확인되지 않거나 하드웨어 오류·통신 오류·사용자 중단이 발생하면 현재 위치 목표로 정지를 시도한다. 자동 토크 해제는 진행하지 않는다.
- 최종 도착 후 ID 11~16의 토크를 해제하고 모두 읽어 확인한다. 일부 실패 시 어느 모터가 확인되지 않았는지 오류를 표시한다. 이 경우 일부 모터는 이미 OFF일 수 있으므로 팔을 받친다.
- 그리퍼 닫힘은 ID 16이 목표에서 20펄스 이내이고 0.5초 정지한 것을 확인한다. 닫힘 실패나 닫힘 후 팔 자세 이탈 시에도 토크를 자동 해제하지 않는다.
- 집기 도중 실패하거나 취소하면 복귀를 자동 실행하지 않는다.
- 모의 테스트는 도착 확인, 지연, 오류, 중단, 손목 경계 통과, 토크 해제 순서를 검사한다. 실제 충돌·받침·집기 성공은 실물 시험이 필요하다.

`--output`을 지정하면 실물 실행에서 복귀 단계, 위치 확인, 토크 해제 결과를 JSONL로 기록한다.
