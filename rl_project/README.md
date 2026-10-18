# WAAM-HyCoS

STL과 config.yaml에서 직접 생성하는 독립 Python Solver. 원본 Validator를 import하거나 실행하지 않으며, 다른 Solver·trajectory.csv를 읽지 않습니다. 공식 PASS 및 최종 성능 평가는 사용자가 수행합니다.

## PowerShell 실행

프로젝트 루트에서 Python 3.11 이상으로 실행합니다.

```powershell
python -m venv rl_project/.venv
.\rl_project\.venv\Scripts\Activate.ps1
python -m pip install -r rl_project/requirements.txt

python -m rl_project.solve --ids 1 2 3 4 5 6 --mode generate
python -m rl_project.solve --ids 5 --mode generate
python -m rl_project.solve --ids 1 2 3 4 5 6 --mode optimize
# 초기해 완성 이후 모델당 탐색 시간과 반복 횟수 제한
python -m rl_project.solve --ids 5 --mode optimize --time-limit 120 --iterations 100

# 작은 테스트와 부분 생성 (공식 Validator를 실행하지 않음)
python -m unittest discover -s rl_project/tests -v
python -m rl_project.solve --ids 1 --mode optimize --max-layers 1 --time-limit 20 --iterations 7
```

활성화가 막히면 `python` 대신 `.\rl_project\.venv\Scripts\python.exe`를 사용합니다. `--input-root`로 동일한 config/STL 계약의 입력 폴더를 지정할 수 있습니다. `--output-root`는 `rl_project/` 안으로 제한됩니다. 모델 번호는 입력/출력 폴더 선택에만 사용됩니다.

## 구현

- **Geometry**: STL 수평 절단, 홀의 even/odd 판정, Polygon/MultiPolygon, 경계 Voronoi 중심선, contour-parallel, 연결 가능한 zigzag/raster, 미피복 중심선 보충. 실제 round-cap 비드 합집합의 Coverage/Overfill/IoU로 후보와 중복 제거를 심사합니다. 적층 길이와 단절 경로의 arc 설정 시간을 함께 줄입니다. 형상+설정 SHA-256 디스크 캐시와 메모리 LRU를 사용합니다.
- **Partition**: 경로 위 가중 Voronoi와 공간 군집, 구간별 Reach 검사, 작업량·이동·병렬성에 따른 배정. 분할은 기존 선분을 나누므로 의도적 적층 면적을 바꾸지 않습니다. 하위 레이어의 비드 영역과 현재 작업의 지지 주변영역이 겹치는 작업으로 Local Support DAG를 만듭니다. 교차 지지영역이 없으면 하위 레이어 전체 완료를 기다립니다.
- **Scheduling**: Greedy 완료시간 배정, 경로 방향·닫힌 루프 시작점 선택, D/T/W 및 arc on/off 대기, 도달 범위 내 park-to-park 동작. Home을 우선 주차점으로 사용하고 불가능하면 Base를 검사합니다. 주차 로봇과의 이동 충돌은 XY 주차점 경유로 복구합니다. 초기해는 충돌 가능한 swept-envelope 작업을 직렬화합니다.
- **Collision**: TCP 선형 상대운동의 최소거리와 Base–TCP XY Capsule의 연속시간 거리 하한 검증. Arm 검사는 swept triangle 필터와 Lipschitz 적응 분할을 사용하며 불확실한 근접 접촉은 충돌로 취급합니다. 모든 D/T/W, 대기·종료 후 주차 상태를 포함합니다. 충돌 시 시간 이동, 다른 배정·순서·방향 후보를 시험합니다. 충돌 쌍과 구간을 기록하고 CP-SAT에 보수적 비중첩 제약으로 전달합니다.
- **ALNS**: Critical relocate, workload balance, conflict reorder, path reorientation, geometry repartition, timing repair, local CP-SAT. 실제 개선량으로 선택 가중치를 갱신합니다. CP-SAT은 최대 14개 작업의 선택적 interval, 배정, 순서, 시작시간과 외부 고정 작업을 모델링합니다. 모든 제안은 실제 연속시간 Scheduler를 다시 통과해야 채택됩니다.
- **효율·보존**: ALNS 변경 전의 위상 순서 prefix를 재사용하고 영향 suffix만 재계산합니다. 최초 연산자는 끝부분에서 시작하고 이후 최대 72개 작업의 neighborhood를 전체 critical chain에서 이동합니다. 재분할은 마지막 3개 레이어에 적용합니다. XY 공간 캐시와 시간차를 포함한 연속 충돌 캐시를 분리합니다. 초기해를 먼저 저장하고 개선해는 Preflight 이후 임시 파일의 원자적 교체로 저장합니다. 실패·시간 초과·중단 시 완성된 이전 CSV를 유지하며 매 반복의 최적화 이력도 저장합니다.
- **CSV**: float32로 좌표를 먼저 확정하고 그 거리로 시간을 계산합니다. 17자리 직렬화로 double round-trip을 유지합니다. 3대 로봇, 0초 시작, 엄격한 시간 증가, 속도·Reach·workspace·Z·D 양의 XY 길이·정지 W·최종 W를 생성 전후에 검사합니다. 저장 파일이 연속시간 검사한 동작과 정확히 같은지도 스트리밍으로 확인합니다.

설계의 핵심은 형상 평가를 통과한 경로 분할과 **주차 상태까지 포함한 충돌 예약**을 연결한 점입니다. 다른 학생 코드는 열람·비교하지 않았으므로 타 구현 대비 성능 우위를 주장하지 않습니다. 강화학습은 사용하지 않습니다.

## 결과

```text
rl_project/results/
  .geometry_cache/             # 형상/설정별 경로 캐시
  01/ ... 06/
    config.yaml                # 원본과 SHA-256 일치
    target.stl                 # 원본과 SHA-256 일치
    trajectory.csv             # 완성된 현재 최적해: 실행 후 생성
    trajectory.initial.csv     # 보수적 초기해
    trajectory.improved.csv    # 현재 최적해 스냅샷 (개선 없으면 초기해와 동일)
    schedule.initial.json
    schedule.improved.json
    inputs.json / best.json / summary.json
    geometry.json / shape_estimate.json
    generation.log / optimization.json / conflict_repairs.json
    smoke/                     # --max-layers 전용, 전체 형상 검증용이 아님
  batch.json
```

`--time-limit`은 초기 형상·초기해 생성 이후의 ALNS 탐색 예산입니다. 실행 중인 개별 geometry/CP 모델 구성/체크포인트 저장은 즉시 중단되지 않아 예산을 소폭 넘길 수 있습니다. 초기해 완성 전 중단해도 디스크 geometry cache는 남습니다. 완성된 trajectory는 모든 작업이 포함된 경우에만 발행됩니다. 여러 실행이 같은 출력 폴더를 동시에 사용하지 않도록 하십시오.

주차 복귀와 swept-envelope CP 제약은 보수적이라 추가 병렬화 여지가 있습니다. 지지 DAG는 비드 영역 기반의 기하 선후관계이며 열·응력·용융풀·실제 관절 운동학은 모델링하지 않습니다. 모든 형상에 대해 해 존재나 전역 최소 Makespan을 보장하지 않습니다. 내부 형상 기준 또는 안전 동작을 찾지 못하면 이유를 기록하고 실패 종료하며 결과를 꾸미지 않습니다. 전체 6개 모델의 공식 Validator 검증과 장시간 벤치마크는 개발 과정에서 실행하지 않습니다.


## Hole 보존 수정 및 형상 디버깅

자기 접촉이 있는 내부 경계가 기존 polygonize 단계에서 누락되는 문제를 수정했습니다.
각 경계 루프를 개별 복구한 뒤 Even-Odd XOR로 합성하여 Hole과 중첩된 섬을 보존합니다.
Contour/Raster/Hybrid의 중심선을 재료 영역으로 제한하며 분리된 조각을 별도 경로로 유지합니다.
비드 반경 전체를 일괄 축소하지 않고 중심선과 실제 round-cap 비드 면적으로 평가합니다.
새로 생성하는 경로는 Coverage 95% 이상을 유지하도록 중복 제거를 제한합니다.
캐시 버전을 변경했으므로 이전의 잘못된 단면 평가 캐시는 재사용하지 않습니다.

```powershell
python -m rl_project.solve --ids 3 --mode generate
python -m pip install -r rl_project/requirements-debug.txt
python -m rl_project.geometry.debug_shape --job rl_project/results/03 --before rl_project/results/03/shape_fix_before/trajectory.csv
python -m unittest discover -s rl_project/tests -v
```

실제 CSV의 D 구간만 float32 좌표로 읽고 반경 4mm 비드를 합집합하여 비교합니다.
`shape_debug.png`는 수정 전/후의 목표, 적층, Overfill, Underfill을 표시하며,
`shape_debug.json`에는 전체 레이어 지표와 Hole 횡단 길이를 저장합니다.
이는 독립적인 경량 형상 검사이며 공식 Validator 실행이나 공식 PASS 판정이 아닙니다.
