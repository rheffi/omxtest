# OMX 손목 회전 수정 (2026-10-20)

## 원인과 변경

기존 자동 IK는 손목(ID 15)을 2048로 계산했지만 HIGH·드랍은 사용자가 기록한 값을 사용했다. 상품 설정에서 기록한 집게 방향은 실제 IK에 쓰이지 않았다.

이 노트북의 실제 기록은 HIGH=32, 드랍 접근=26, 드랍 놓기=4069다. 기존 모드 4 제어에서 `26 → 4069`를 숫자 그대로 보간하면 약 355도 회전한다. 수정본은 현재 측정값에 가까운 동등 위치를 선택하여 `26 → -27`로 약 4.7도 이동한다. 복귀도 같은 방식이다. 기존 저장 자세 파일은 바꾸지 않는다.

- 상품 설정에 남아 있는 실제 집기 자세에서 모델 집게 +Y축의 수평 방향을 읽어 접근·호버·집기·상승에 사용한다. 새 설정은 상품별 `grasp_reference_pulses`도 저장한다.
- 베이스 회전 시 같은 수평 집게 방향을 유지하도록 ID 15를 함께 계산한다. 손목을 바꾼 뒤의 작은 말단 오프셋도 FK에 포함해 위치를 다시 계산한다.
- 앞 단계에 가까운 관절 해를 선택하고, 계산은 실제 실행 순서대로 한다. 퇴각은 계산한 접근·호버 자세를 재사용한다.
- ID 15만 가까운 동등 회전값을 사용한다. ID 11~14는 기존 절대 위치 경로를 유지한다.
- ID 15의 소프트웨어 허용 창은 -4096..8191이다. 하드웨어의 전체 확장 범위를 허용하지 않으며, 정확히 180도 차이인 모호한 손목 목표는 차단한다. 이 범위 자체가 케이블·주변 간섭을 검증한 것은 아니다.
- ID 15가 음수 위치에 있어도 다음 실행·자세 기록이 가능하다. 저장 자세·집기 기록에는 동등한 0..4095 값을 저장하고 실제 전송 때 현재 회전수에 맞춘다.

모터 모드·Homing Offset·전류 설정, 캘리 행렬, 기존 상품 설정, 저장 자세는 이 패치로 변경하지 않는다. 모드 4의 음수 Goal Position은 [ROBOTIS XL330-M288 문서](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#goal-position116)에서 확인할 수 있다.

## 적용

현재 실행 폴더에 이미 수정본이 적용됐다면 이 절은 생략한다. 다른 노트북에서는 패치 ZIP을 Downloads에 복사한 뒤 다음을 실행한다. 원래 전달 ZIP 전체를 실행 폴더 위에 다시 풀지 않는다.

```bash
unzip -o ~/Downloads/omx_wrist_direction_fix_20261020.zip -d /tmp/omx-wrist-fix-20261020
```

```bash
python3 /tmp/omx-wrist-fix-20261020/apply_wrist_fix.py --target ~/dev_ws/omxtest
```

적용 도구는 원본 코드 체크섬을 먼저 확인하고, 기존 코드를 `backups/wrist_fix_*` 아래에 저장한다. 다른 수정이 있으면 덮어쓰지 않고 중단한다. `config/`와 기존 `records/`는 변경하지 않는다. 완료 후 출력한 백업 경로를 보관한다.

## 계산 확인 → 빈 집게 확인 → 실제 집기

```bash
cd ~/dev_ws/omxtest
```

장비 연결 없이 기록한 집기점의 모델 경로를 계산한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --wrist-preview
```

`wrist_path`에서 `id15_pulse`는 저장·계산한 한 바퀴 기준값이고, `id15_unwrapped_pulse`는 HIGH를 기준으로 연결한 짧은 이동 목표다. 실제 전송은 매 구간 현재 측정값을 기준으로 결정한다. 음수 값 자체는 오류가 아니다. 이 명령은 기록한 집기점의 계산만 확인하며 현재 상품 검출·모터 상태·실제 도착은 확인하지 않는다.

팔을 기록한 HIGH에 놓고 상품을 카메라에 인식시킨 뒤 빈 집게로 호버한다. `MOVE` 입력 후 동작한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --hover --live --stage-seconds 3
```

빈 집게로 저장된 드랍 경로를 따로 확인한다. 이 명령은 그리퍼를 여닫지 않는다. 실제 바구니 위치와 경로를 확인하고 `MOVE`를 입력한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --drop-test --live --stage-seconds 3
```

두 경로를 실물로 확인한 뒤 상품을 놓고 기존 집기를 실행한다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/picking/omx_pick.py --live
```

HIGH·드랍의 집게 방향은 저장 자세에 따라 전환된다. 그 전환도 없애고 싶다면 팔을 받치고 기존 토크 해제 명령으로 수동 조절한 뒤, 바꾸려는 자세만 아래 명령으로 읽어 `REPLACE`한다. 아래 명령이 팔을 이동시키지는 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save hover_high --replace
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save drop_approach --replace
```

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B tools/measurement/07_omx_pose_recorder.py --save drop_release --replace
```

## 확인 범위

자동 검증 명령은 카메라·모터를 열지 않는다.

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-pick/bin/python -B -m unittest discover -s tests -v
```

기록 방향 재사용, 베이스 회전 보상, 수직 이동 중 방향, 0/4095 경계의 실제 전송 목표, 클릭 호버, 기존 설정 읽기, 오프라인 미리보기 등을 검증한다. 이 노트북의 좌우 선반 모서리와 중앙에서 3개 높이씩 총 30개 모델 목표도 계산했다. 최대 위치 오차 약 0.34mm, 방향 오차 약 0.05도였다. 이는 모델 계산 결과이며 실물 파지·경로의 정확도나 성공을 뜻하지 않는다. 실물 모터 시험은 자동으로 수행하지 않았다.
