# competition_runtime — 대회 실행본 중 본인 작성 source

[상위 폴더 README](../../README.md) · [`../portfolio_2026/` (재구성본)](../portfolio_2026)

이 폴더는 대회 당시 제가 패키징한 **CYH_drive V4 dual-overlay** 실행본에서, **출처가 제(CYH)로 확인되는 source 파일만** 추린 것입니다. 바로 위 `src/portfolio_2026/`가 보드·DPU 없이 PC에서 테스트할 수 있도록 **다시 작성한 재구성본**이라면, 이 `competition_runtime/`은 **대회 실행본의 실제 파일 자체**(재작성 아님)입니다.

## `portfolio_2026/`와의 차이

| | `portfolio_2026/` (재구성) | `competition_runtime/` (실제 실행본) |
|---|---|---|
| 성격 | 보드 의존성 제거 + unittest 가능한 clean-room 재작성 | 대회 패키지의 원본 파일 추출 |
| 실행 | PC에서 `python -m unittest` 통과 | **단독 실행 불가** — PYNQ 보드·DPU·팀원 모듈 필요 |
| 담은 것 | 차선 미검출 처리, 모드 자원 수명주기, 녹화 로직의 재구성 | dual-overlay 런처, ADC/PWM 조향·모터 제어, 캘리브레이션 시스템, 무동작 검증, 노트북 |
| 검증 | 외부 패키지 없이 테스트 | `python -m py_compile` (문법만; 하드웨어 동작 아님) |

> **대회 당일 실행본과 동일한지**: 이 파일들은 제가 만든 최종 패키지(V4 dual-overlay)이지만, 대회 당일 실제로 돌린 bit/주소/파라미터와 **byte 단위로 동일한지는 독립적으로 재확인하지 못했습니다.** 설계·구조를 보여주기 위한 자료로 읽어 주세요.

## 포함 (CYH 작성으로 확인)

- `main.py` — 자율/수동/주차를 **별도 자식 프로세스**로 띄우는 dual-overlay 런처 (bit별 MMIO 주소 격리)
- `drive_app.py`, `parking_app.py` — 각 모드의 진입·자원 로드·종료(Q/X) 수명주기
- `config.py` — schema v2 설정 (하드웨어 주소 맵, 캘리브레이션 로드, 모드별 프로파일)
- `motor_controller.py` — **ADC 피드백 기반 100% duty 펄스 조향 모델**과 PWM 모터 제어 (PJH baseline 140줄 → 459줄로 재작성)
- `calibration_io.py`, `calibration_helpers.py`, `calibration_result.example.json` — 조향 ADC 3점 측정·보정 시스템과 예시 결과(합성값)
- `verify_profiles.py` — 하드웨어 없이 프로파일을 점검하는 무동작 검증
- `CALIBRATION_TEAM08_FULL.ipynb`, `RUN_TEAM08.ipynb` — 보정/실행 노트북 (**출력 셀 제거**)

## 제외 (팀원·교육 baseline·권리/보안)

- `image_processor.py` — 차선 fitting·dynamic offset 등 **PJH 작성** 인식 로직
- `driving_system_controller.py` — Stanley·CTE 등 **PJH 작성** 제어 핵심 (상위 README 10절 근거)
- `parking_mission.py` — 주차 시퀀스 (상위 README에서 **팀장 구현**으로 귀속)
- `yolo_utils.py`, `AutoLab_lib.py` — SKKU AutomationLab 교육 baseline과 **byte-identical** (diff 0)
- `dpu.bit`/`hwh`/`xclbin`/`xmodel`, 모델, 학습 dataset, 주행 영상, IDE 메타데이터

이 때문에 `main.py`/`drive_app.py`의 자율주행 경로는 위 제외 모듈을 import 하므로 이 폴더만으로는 실행되지 않습니다. 이는 **팀원 저작물을 그대로 재배포하지 않기 위한 의도된 제외**이며, 제가 맡은 녹화·차선 미검출·모드 수명주기 로직의 실행 가능한 형태는 `../portfolio_2026/`에 있습니다.

## 실행 환경 (참고)

PYNQ 보드에서 `pynq`, `pynq_dpu`, `spidev`, `keyboard`, `opencv`가 필요합니다. 데스크톱에서는 `python -m py_compile <파일>`로 문법만 확인됩니다.

## Attribution

팀 공동 개발입니다. 포함한 파일은 출처가 제(CYH)로 확인되는 범위이며, 인식(PJH)·Stanley 제어(PJH)·주차(팀장) 핵심과 교육 baseline은 포함하지 않습니다. 공동 연구/팀 source를 단독 저작물로 주장하지 않습니다.

---

## English summary

`competition_runtime/` holds the **actual competition package files** (CYH_drive V4 dual-overlay) whose authorship is confirmed as mine, as opposed to `../portfolio_2026/`, which is a board-free, unit-tested **clean-room reconstruction**. Included: the dual-overlay launcher (each mode in a separate child process with isolated MMIO), ADC/PWM steering + motor control, the calibration system, no-op profile verification, and the calibration/run notebooks (outputs cleared). Excluded, by authorship and licensing: teammate PJH's lane-fitting (`image_processor.py`) and Stanley/CTE control (`driving_system_controller.py`), the team lead's parking implementation (`parking_mission.py`), the SKKU AutomationLab educational baseline (`yolo_utils.py`/`AutoLab_lib.py`, byte-identical to the course files), and all bitstreams/models/datasets/videos. Because of these exclusions the autonomous path is **not runnable from this folder**; the runnable form of my recording/lane-loss/mode-lifecycle logic lives in `../portfolio_2026/`. I do not verify byte-for-byte identity with what ran on competition day, and I do not claim joint/team source as sole authorship.
