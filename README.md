# WAAM-HyCoS

### 다중 로봇 WAAM 공정의 형상 기반 경로 생성 및 충돌 인지형 스케줄링 최적화

**WAAM-HyCoS (Hybrid Geometry and Conflict-aware Scheduling Optimization)**는 3대의 산업용 로봇이 협업하는 WAAM(Wire Arc Additive Manufacturing) 공정에서 적층 경로를 자동으로 생성하고, 충돌을 회피하면서 전체 공정 완료시간(Makespan)을 최소화하기 위해 개발한 하이브리드 최적화 시스템이다.

STL 형상 분석부터 적층 경로 생성, 로봇별 작업 분할, 작업 선후관계 구성, 초기 스케줄 생성, ALNS 기반 스케줄 개선까지 하나의 파이프라인으로 통합하였다.

본 프로젝트의 독립적인 Solver는 `rl_project/` 디렉터리에 구현되어 있으며, 기존 WAAM Validator의 소스코드를 수정하지 않고 검증 환경으로 활용한다.

---

## 1. 프로젝트 목표

본 프로젝트는 다음 조건을 만족하는 다중 로봇 WAAM 작업 스케줄을 생성하는 것을 목표로 한다.

- **형상 정확도:** STL 목표 형상에 적합한 적층 경로 생성
- **로봇 도달 가능성:** 각 로봇의 작업 반경 제약 만족
- **작업 선후관계:** 적층에 필요한 지지관계 및 선행 작업 조건 고려
- **충돌 안전성:** 로봇 TCP 및 Arm Envelope 충돌 회피
- **병렬 작업:** 로봇 간 불필요한 대기 감소 및 동시 작업 확대
- **Makespan 최적화:** Greedy, ALNS 및 Local CP-SAT을 활용한 공정 완료시간 개선

전체 최적화 목적함수는 다음과 같다.

\[
\min_S C_{\max}(S)
\]

\[
C_{\max}(S)=\max\{C_1(S),C_2(S),C_3(S)\}
\]

여기서 \(C_r(S)\)는 스케줄 \(S\)에서 로봇 \(r\)의 완료시간을 의미한다.

---

## 2. 알고리즘 구성

WAAM-HyCoS는 다음과 같은 처리 단계로 구성된다.

```text
STL + config.yaml
        |
        v
[1] STL 형상 분석 및 슬라이싱
        |
        v
[2] 적층 경로 생성 및 형상 평가
        |
        v
[3] 로봇별 작업 분할
        |
        v
[4] Local Support DAG 구성
        |
        v
[5] 충돌 인지형 Greedy 초기 스케줄 생성
        |
        v
[6] ALNS + Local CP-SAT 스케줄 개선
        |
        v
[7] 충돌 및 형상 자체 검사
        |
        v
trajectory.csv 생성
        |
        v
공식 WAAM Validator 검증
```

### 2.1 형상 기반 적층 경로 생성

STL 모델을 적층 높이별로 슬라이싱하고, 각 단면의 기하학적 특성을 고려하여 로봇이 수행할 경로를 생성한다.

주요 기능은 다음과 같다.

- STL 단면 추출
- Contour, Raster, Centerline 기반 경로 생성
- Even-Odd 방식의 내부 경계 복원
- Hole 및 복잡한 경계 처리
- 적층 경로의 기하학적 보정
- Coverage, Overfill, IoU 기반 자체 형상 평가

### 2.2 로봇별 작업 분할

Weighted Voronoi 기반 공간 분할을 적용하여 로봇별 작업을 배정한다.

로봇의 설치 위치, 작업점까지의 거리, 도달 가능 범위 및 작업 부하를 고려하여 작업 영역을 결정한다.

### 2.3 Local Support DAG

적층 작업 간 선후관계를 방향성 비순환 그래프(DAG)로 구성한다.

전체 레이어의 동기화를 강제하는 대신, 필요한 국소 지지관계를 기준으로 후속 작업의 실행 가능 시점을 판단하여 병렬 작업 기회를 확대한다.

### 2.4 Greedy 초기 스케줄링

다음 요소를 고려하여 초기 스케줄을 생성한다.

- 작업별 예상 소요시간
- 로봇별 작업 가능 시점
- 로봇별 누적 작업 부하
- 작업 선후관계
- 충돌 가능성 및 대기시간

### 2.5 충돌 인지형 스케줄링

로봇 TCP 간 거리 및 Arm Envelope를 기반으로 작업 수행 중 충돌 가능성을 검사한다.

충돌 가능성이 있는 작업 배치는 제한하거나 시작 시점을 조정하여 스케줄을 구성한다.

### 2.6 Adaptive Large Neighborhood Search (ALNS)

초기 스케줄을 개선하기 위해 7개의 탐색 연산자를 사용한다.

| 연산자 | 기능 |
|---|---|
| Critical Relocate | Makespan에 영향을 주는 작업 재배치 |
| Workload Balance | 로봇별 작업 부하 균형 조정 |
| Conflict Reorder | 충돌 및 대기를 고려한 작업 순서 변경 |
| Path Reorientation | 적층 경로의 진행 방향 조정 |
| Geometry Repartition | 형상 기반 작업 영역 재분할 |
| Timing Repair | 작업 시작 시점 및 대기시간 조정 |
| Local CP-SAT | 제한된 작업 집합에 대한 수리적 최적화 |

ALNS는 연산자별 개선 결과를 반영하여 탐색 가중치를 조정하며, Local CP-SAT을 통해 일부 작업 집합에 대한 추가적인 최적화를 수행한다.

---

## 3. 개발 환경

| 항목 | 내용 |
|---|---|
| 프로그래밍 언어 | Python |
| 권장 Python 버전 | 3.11 이상 |
| 운영체제 | Windows (PowerShell 실행 예시 제공) |
| 대상 로봇 | 산업용 로봇 3대 |
| 입력 데이터 | STL, YAML |
| 경로 생성 | Geometry Optimization |
| 초기 스케줄링 | Greedy |
| 스케줄 개선 | ALNS |
| 국소 최적화 | CP-SAT |
| 검증 도구 | WAAM Validator v1.0.1 |
| 자체 개발 코드 | `rl_project/` |

본 구현은 강화학습을 사용하지 않으며, 기하학적 경로 생성과 탐색 기반 최적화를 결합한 방식이다.

---

## 4. 설치 및 실행 방법

모든 명령어는 **프로젝트 최상위 디렉터리**에서 PowerShell로 실행한다.

### 4.1 프로젝트 다운로드

```powershell
git clone https://github.com/dongtak2/WAAM-HyCoS-.git
cd WAAM-HyCoS-
```

Git을 사용하지 않을 경우 GitHub의 **Code → Download ZIP** 기능을 통해 다운로드할 수 있다.

### 4.2 Python 가상환경 생성

```powershell
python -m venv rl_project/.venv
```

가상환경의 Python 실행 파일은 다음 위치에 생성된다.

```text
rl_project/.venv/Scripts/python.exe
```

### 4.3 라이브러리 설치

PowerShell 실행 정책 문제를 피하기 위해, 가상환경을 활성화하지 않고 Python 실행 파일을 직접 사용한다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m pip install -r rl_project/requirements.txt
```

### 4.4 공개 모델 6개 전체 경로 생성

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 2 3 4 5 6 --mode generate
```

`generate` 모드는 형상 기반 적층 경로와 초기 스케줄을 생성한다.

### 4.5 특정 모델만 실행

예를 들어 모델 03만 실행하려면 다음과 같이 입력한다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 3 --mode generate
```

모델 05만 실행하는 경우:

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 5 --mode generate
```

### 4.6 ALNS 최적화 실행

초기해를 생성한 다음 개선 탐색을 수행하는 모드이다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 2 3 4 5 6 --mode optimize
```

특정 모델의 탐색시간과 반복 횟수를 설정할 수도 있다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 5 --mode optimize --time-limit 120 --iterations 100
```

위 명령어는 모델 05에 대해 ALNS 탐색시간 상한을 120초, 반복 횟수 상한을 100회로 설정한다. 탐색시간은 초기해 생성 이후의 최적화 탐색에 대한 설정이다.

### 4.7 부분 실행 테스트

전체 모델을 생성하기 전에 일부 레이어만 실행하여 기능을 점검할 수 있다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 --mode optimize --max-layers 1 --time-limit 20 --iterations 7
```

`--max-layers` 옵션으로 생성한 부분 결과는 `smoke/` 디렉터리에 저장되며, 전체 STL 형상의 공식 검증을 대체하지 않는다.

---

## 5. 주요 실행 옵션

| 옵션 | 설명 | 기본값 |
|---|---|---|
| `--ids` | 실행할 공개 모델 번호 | 1~6 전체 |
| `--mode` | `generate` 또는 `optimize` | generate |
| `--time-limit` | 모델별 ALNS 탐색시간 상한(초) | 60 |
| `--iterations` | ALNS 최대 반복 횟수 | 80 |
| `--seed` | 난수 시드 | 42 |
| `--max-layers` | 부분 레이어 실행 | 전체 |
| `--input-root` | 입력 모델 폴더 | tests |
| `--output-root` | 출력 폴더 | rl_project/results |
| `--chunk-length` | 작업 경로 분할 관련 길이(mm) | 650 |

`--output-root`는 `rl_project/` 내부 경로만 허용한다.

---

## 6. 입력 및 출력 데이터

### 6.1 입력 파일

각 모델의 실행을 위해 다음 파일이 필요하다.

| 파일 | 역할 |
|---|---|
| `config.yaml` | 로봇, 공정 및 형상 관련 설정 |
| `target.stl` | 목표 적층 형상 |

기본 입력 경로는 프로젝트 최상위 `tests/` 디렉터리이다.

별도의 입력 경로를 사용하려면 `--input-root` 옵션을 지정할 수 있다. 다만 입력 폴더는 Solver에서 요구하는 모델별 구조와 설정 형식을 만족해야 한다.

### 6.2 출력 파일

기본 출력 경로는 다음과 같다.

```text
rl_project/
└── results/
    ├── 01/
    ├── 02/
    ├── 03/
    ├── 04/
    ├── 05/
    └── 06/
```

각 모델 폴더에는 실행 모드에 따라 다음과 같은 파일이 생성된다.

| 파일 | 설명 |
|---|---|
| `config.yaml` | 검증에 사용할 공정 설정 |
| `target.stl` | 검증에 사용할 목표 형상 |
| `trajectory.csv` | 최종 선택된 적층 궤적 |
| `trajectory.initial.csv` | 초기 스케줄 궤적 |
| `trajectory.improved.csv` | 개선 결과 궤적 |
| `schedule.initial.json` | 초기 스케줄 정보 |
| `schedule.improved.json` | 개선 스케줄 정보 |
| `summary.json` | 실행 결과 요약 |
| `shape_estimate.json` | 자체 형상 평가 결과 |
| `optimization.json` | 최적화 실행 정보 |

실행 모드 및 처리 단계에 따라 일부 파일은 생성되지 않을 수 있다.

---

## 7. 공식 WAAM Validator 검증 방법

본 프로젝트에서 가장 중요한 최종 검증은 **공식 WAAM Validator를 통해 생성된 적층 궤적의 유효성을 확인하는 것**이다.

Solver 내부의 자체 검증을 통과했다고 해서 공식 Validator의 PASS가 보장되는 것은 아니다.

### 7.1 검증용 결과 생성

먼저 검증할 모델의 전체 적층 경로를 생성한다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 --mode generate
```

정상적으로 실행되면 다음 경로에 검증용 파일이 준비된다.

```text
rl_project/results/01/
    config.yaml
    target.stl
    trajectory.csv
```

### 7.2 공식 Validator에 입력

원본 WAAM Validator의 공식 검증 환경을 실행한다.

Validator가 제공하는 입력 폴더 선택 기능을 이용하여 다음 디렉터리를 지정한다.

```text
rl_project/results/01/
```

해당 디렉터리에는 다음 세 파일이 모두 존재해야 한다.

1. `config.yaml`
2. `target.stl`
3. `trajectory.csv`

세 파일이 동일한 모델의 입력과 출력으로 구성되어 있는지 확인한 후 Validation을 실행한다.

### 7.3 검증 항목

공식 Validator에서는 다음과 같은 조건을 확인한다.

| 검증 항목 | 주요 확인 내용 |
|---|---|
| 입력 유효성 | 설정, STL 및 궤적 파일의 형식과 일관성 |
| 적층 형상 | Coverage, Overfill, IoU 및 레이어별 형상 |
| 로봇 도달 범위 | 로봇별 XY 작업 반경 |
| 충돌 | TCP 및 Arm Envelope 충돌 |
| 스케줄 | 작업시간, 이동시간, 대기시간 및 Makespan |

검증 결과가 `PASS`로 표시되면 해당 Validator 버전에서 정의한 검증 조건을 만족한 것이다.

반대로 `FAIL` 또는 `ERROR`가 발생한 경우, 검증 보고서와 로그를 통해 실패 원인을 확인해야 한다.

### 7.4 검증 결과 확인

공식 Validator의 실행 결과에는 환경 설정에 따라 다음과 같은 파일이 포함될 수 있다.

```text
summary.json
validation_report.md
robot_metrics.csv
collision_events.csv
layer_metrics.csv
run.log
validation_inputs.json
```

주요 확인 대상은 다음과 같다.

- **Overall Result:** PASS 또는 FAIL
- **Makespan:** 전체 작업 완료시간
- **Robot XY Reach:** 로봇 작업 반경 위반 여부
- **Collision:** TCP 및 Arm Envelope 충돌 건수
- **Shape:** Coverage, Overfill, IoU 및 실패 레이어

공식 검증 결과는 해당 실행에서 실제 생성된 보고서를 기준으로 판단한다.

---

## 8. 자체 검증 및 테스트

공식 Validator와 별개로 Solver 내부 기능을 점검하기 위한 테스트가 제공된다.

### 8.1 단위 테스트

```powershell
.\rl_project\.venv\Scripts\python.exe -m unittest discover -s rl_project/tests -v
```

이 테스트는 Solver의 내부 기능을 검사하기 위한 것으로, 공식 Validator의 PASS 판정을 대체하지 않는다.

### 8.2 적층 형상 자체 평가

자체 형상 분석에 필요한 추가 의존성을 설치한다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m pip install -r rl_project/requirements-debug.txt
```

예를 들어 모델 03의 생성 결과를 분석하려면 다음 명령어를 사용한다.

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.geometry.debug_shape --job rl_project/results/03
```

자체 분석 결과는 모델 폴더에 다음 파일로 저장된다.

```text
shape_debug.json
shape_debug.png
```

`shape_debug.json`은 레이어별 형상 지표와 전체 평가 결과를 기록하며, `shape_debug.png`는 목표 형상과 생성 경로를 시각적으로 비교한다.

이 검사는 경량 형상 분석을 위한 독립 평가이며, 공식 WAAM Validator 실행 결과가 아니다.

---

## 9. 공개 벤치마크 검증 결과

WAAM-HyCoS를 공개 6개 벤치마크 모델에 적용하고 공식 WAAM Validator v1.0.1에서 검증하였다.

| 모델 | Makespan (초) | 공식 Validator |
|---|---:|---|
| 01 Honeycomb | 1,528.94 | PASS |
| 02 Spiral | 2,874.25 | PASS |
| 03 Triangular Truss | 2,364.06 | PASS |
| 04 Motor Inner | 72,865.85 | PASS |
| 05 Twisted Shell | 234,936.33 | PASS |
| 06 Angle Frame | 7,721.85 | PASS |

### 모델 01 검증 사례

모델 01에서는 다음과 같은 결과를 확인하였다.

| 지표 | 결과 |
|---|---:|
| Makespan | 1,528.94초 |
| Coverage | 92.07% |
| Overfill | 0.54% |
| IoU | 91.57% |
| Arm Envelope 충돌 | 0건 |
| TCP 충돌 | 0건 |
| 실패 레이어 | 0개 |
| 최종 검증 | PASS |

### 결과 해석 시 주의사항

위 수치는 공개 벤치마크에서 공식 검증을 통과한 실행 결과이다.

- PASS는 Validator에서 정의한 조건을 만족했다는 의미이며, 수학적인 전역 최적해를 의미하지 않는다.
- 위 검증 결과만으로 ALNS가 Greedy 대비 Makespan을 얼마나 단축했는지 단정할 수 없다.
- 실제 물리 시스템의 안전성을 보장하는 것은 아니며, Validator가 정의한 로봇 및 충돌 모델의 범위 내에서 검증한 결과이다.
- 새로운 STL 형상이나 다른 로봇 설정에 대해서는 별도의 검증이 필요하다.

---

## 10. 프로젝트의 주요 특징

### 형상 생성과 스케줄링의 통합

목표 STL 모델을 입력으로 받아 적층 경로와 로봇별 작업 스케줄을 함께 생성한다.

### 다중 로봇 병렬 작업 고려

작업 선후관계와 로봇 간 충돌을 함께 고려하여 병렬 수행 가능한 작업을 탐색한다.

### 하이브리드 최적화 구조

Greedy 기반 초기해에 ALNS 및 Local CP-SAT을 결합하여 다양한 스케줄 개선 방안을 탐색한다.

### 재현 가능한 검증 절차

입력 파일, 생성된 적층 궤적, 검증 항목 및 공식 Validator 실행 과정을 구분하여 기록한다.

---

## 11. 한계 및 향후 개선 방향

현재 구현의 한계와 향후 개선 과제는 다음과 같다.

1. Greedy 대비 ALNS 및 Local CP-SAT의 정량적인 성능 개선 효과를 추가 검증해야 한다.
2. 대규모 STL 모델에서 형상 처리, 충돌 검사 및 스케줄 재구성의 계산시간을 줄일 필요가 있다.
3. 복잡한 내부 Hole 및 자기 접촉 경계를 가진 형상에 대한 안정성을 지속적으로 개선해야 한다.
4. 실제 WAAM 공정에서 발생하는 열변형, 냉각시간, 용융풀 변화 등 물리적 요소를 추가 고려할 필요가 있다.
5. 공개 6개 모델 외 다양한 형상에 대한 일반화 성능 평가가 필요하다.

최종적으로는 형상 정확도와 충돌 안전성을 유지하면서 Makespan을 단축할 수 있는 범용적인 다중 로봇 WAAM 최적화 시스템으로 발전시키고자 한다.

---

## 12. 원본 프로젝트 및 라이선스

본 프로젝트는 WAAM Validator를 검증 환경으로 활용하며, 별도의 `rl_project/` 디렉터리에 독립적인 Solver를 개발하였다.

원본 WAAM Validator의 저작권 및 라이선스는 해당 저작권자에게 귀속된다.

프로젝트 공개 및 재배포 시 원본 저장소의 라이선스 조건을 확인해야 한다.
