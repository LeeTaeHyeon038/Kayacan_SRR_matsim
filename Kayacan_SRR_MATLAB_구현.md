# Kayacan et al. (2012) MATLAB/Simulink 구현 설명

## 개요

이 문서는 Kayacan et al. (2012) 논문 *"Modeling and control of a spherical rolling robot: a decoupled dynamics approach"* 의 시뮬레이션을 MATLAB/Simulink로 구현한 내용을 설명한다.

구현 목표는 다음과 같다:
- 논문의 운동방정식 및 제어 알고리즘을 코드로 재현
- PD 제어기와 PD-Fuzzy 제어기(PDFC) 성능 비교
- 직선 및 곡선 궤적 추종 시뮬레이션

---

## 폴더 구조

```
Kayacan_SRR_matsim/
│
├── run_all.m              ← 마스터 실행 스크립트 (이것만 실행하면 됨)
├── main_sim.m             ← ode45 기반 단독 시뮬레이션
│
├── core/                  ← 핵심 수식/파라미터 함수
│   ├── params.m
│   ├── get_params.m
│   ├── calc_M.m
│   ├── calc_V.m
│   ├── control_input.m
│   ├── dynamics.m
│   ├── traj_linear.m
│   └── traj_circular.m
│
├── models/                ← Simulink 모델
│   ├── SRR_PD.slx
│   └── SRR_PDFC.slx
│
├── simulations/           ← 시뮬레이션 실행 스크립트
│   ├── run_linear.m
│   ├── run_linear_PDFC.m
│   ├── run_curv_PD.m
│   └── run_curv_PDFC.m
│
├── plots/                 ← 결과 플롯 스크립트
│   ├── plot_results.m
│   ├── plot_linear_comparison.m
│   ├── plot_curvilinear.m
│   └── plot_curvilinear_comparison.m
│
└── results/               ← 시뮬레이션 결과 저장
    ├── results_linear.mat
    ├── results_linear_PDFC.mat
    ├── results_curv_PD.mat
    └── results_curv_PDFC.mat
```

---

## 실행 방법

### 전체 자동 실행

MATLAB에서 `Kayacan_SRR_matsim` 폴더를 Current Folder로 설정한 뒤:
먼저 `SRR_PD.slx`, `SRR_PDFC.slx`의 `Traj_gen` 블록 코드를 `traj_linear`인지 확인한다. 만약 `traj_circular`로 되어 있다면 그 부분을 `traj_linear`로 변경한다.

그 다음에: 
```matlab
run_all
```

실행 중간에 두 번 멈추는 구간이 있다.

**첫 번째 pause**: 직선 궤적 시뮬레이션 완료 후
→ `SRR_PD.slx`, `SRR_PDFC.slx`의 `Traj_gen` 블록 코드를 `traj_circular`로 변경하고 저장 후 Enter

**두 번째 pause**: 곡선 궤적 시뮬레이션 완료 후
→ `Traj_gen` 블록 코드를 다시 `traj_linear`로 복원하고 저장 후 Enter

### 개별 실행

특정 시뮬레이션만 돌리려면:

```matlab
addpath('core'); addpath('models'); addpath('simulations'); addpath('plots'); addpath('results');
run_linear          % 직선 PD
run_linear_PDFC     % 직선 PDFC
plot_results        % 결과 플롯
```

---

## 핵심 수식 구현

### 1. 시스템 파라미터 (`core/params.m`, `core/get_params.m`)

논문 Section 5의 수치값:

| 변수 | 설명 | 값 |
|---|---|---|
| $M_s$ | 구의 질량 | 3 kg |
| $m_p$ | 진자의 질량 | 2 kg |
| $R$ | 구의 반지름 | 0.2 m |
| $l$ | 진자의 길이 | 0.075 m |
| $I^s$ | 구의 관성모멘트 | $(2/3)M_sR^2$ |
| $I^p$ | 진자의 관성모멘트 | $(1/3)m_pl^2$ |

`params.m`은 workspace에 변수를 올리는 스크립트이고, `get_params.m`은 Simulink MATLAB Function 블록 내부에서 파라미터를 구조체로 반환하는 함수다.

---

### 2. 관성 행렬 M(q) (`core/calc_M.m`)

논문 식 (31), (32)의 $M(\mathbf{q})$ 행렬 계산.

일반좌표는 $\mathbf{q} = [\theta, \alpha, \phi, \beta]^T$이다:
- $\theta$: x축 기준 구 회전각
- $\alpha$: x축 기준 진자 회전각
- $\phi$: y축 기준 구 회전각
- $\beta$: y축 기준 진자 회전각

디커플링 가정에 의해 M은 블록 대각 구조를 가진다:

$$M = \begin{bmatrix} M_{11} & M_{12} & 0 & 0 \\ M_{21} & M_{22} & 0 & 0 \\ 0 & 0 & M_{33} & M_{34} \\ 0 & 0 & M_{43} & M_{44} \end{bmatrix}$$

x-subsystem 성분:

$$M_{11} = M_sR^2 + m_pR^2 + m_pl^2 + I^s + I^p + 2m_pRl\cos(\alpha-\theta)$$
$$M_{12} = M_{21} = -m_pl^2 - I^p - m_pRl\cos(\alpha-\theta)$$
$$M_{22} = m_pl^2 + I^p$$

y-subsystem도 동일한 구조 ($\theta \to \phi$, $\alpha \to \beta$).

```matlab
function M = calc_M(q, params)
% q = [theta; alpha; phi; beta]
% params = 파라미터 구조체
```

---

### 3. 비선형 항 V(q, dq) (`core/calc_V.m`)

논문 식 (31), (32)의 $V(\mathbf{q}, \dot{\mathbf{q}})$ 벡터 계산.

x-subsystem의 구(theta) 방정식 비선형 항:

$$V_1 = m_pRl\sin(\alpha-\theta)[\dot\theta^2 + \dot\alpha^2 - 2\dot\theta\dot\alpha] - m_pgl\sin(\alpha-\theta)$$

x-subsystem의 진자(alpha) 방정식 비선형 항:

$$V_2 = m_pgl\sin(\alpha-\theta)$$

y-subsystem도 동일한 구조.

세 가지 물리적 의미:
- $\dot{q}^2$ 항: 원심력 효과
- $\dot{q}_1\dot{q}_2$ 항 (계수 2): 코리올리 힘
- $\sin$ 항: 중력에 의한 복원력

---

### 4. 피드백 선형화 제어 입력 (`core/control_input.m`)

논문 식 (41):

$$\mathbf{u} = V(\mathbf{q}, \dot{\mathbf{q}}) + K_v\dot{\mathbf{e}} + K_p\mathbf{e} + M(\mathbf{q})\ddot{\mathbf{q}}_d$$

여기서 오차는 $\mathbf{e} = \mathbf{q}_d - \mathbf{q}$, $\dot{\mathbf{e}} = \dot{\mathbf{q}}_d - \dot{\mathbf{q}}$.

이를 운동방정식 $M\ddot{\mathbf{q}} + V = \mathbf{u}$에 대입하면 오차 동역학:

$$\ddot{\mathbf{e}} + K_v\dot{\mathbf{e}} + K_p\mathbf{e} = 0$$

$V$가 상쇄되고 $M$이 상쇄되어 완전한 선형화가 달성된다. 즉 비선형 시스템이 선형 2차 시스템처럼 동작한다.

---

### 5. 상태방정식 (`core/dynamics.m`)

ode45에 넘길 상태방정식. 상태벡터는 $[\mathbf{q}; \dot{\mathbf{q}}]$ (8×1).

$$\ddot{\mathbf{q}} = M(\mathbf{q})^{-1}(\mathbf{u} - V(\mathbf{q}, \dot{\mathbf{q}}))$$

코드에서 `M \ (u - V)`를 쓰는 이유: `inv(M) * (u-V)`보다 수치적으로 더 안정적이기 때문.

---

### 6. 목표 궤적 생성

#### 직선 궤적 (`core/traj_linear.m`)

목표 선속도 $v_d = 0.5$ m/s. 구 반지름 $R = 0.2$ m이므로 목표 각속도 $\dot\theta_d = v_d/R = 2.5$ rad/s.

$$\mathbf{q}_d(t) = [2.5t,\ 0,\ 0,\ 0]^T$$
$$\dot{\mathbf{q}}_d = [2.5,\ 0,\ 0,\ 0]^T$$
$$\ddot{\mathbf{q}}_d = [0,\ 0,\ 0,\ 0]^T$$

#### 원형 궤적 (`core/traj_circular.m`)

곡률 반경 $e = 1.0$ m의 원형 궤적. 논문 식 (39)에서 $\beta$ 역산:

$$e = \frac{R\dot\theta^2(I^s - m_pRl\cos\beta + R^2(M_s+m_p))}{m_pgl\sin\beta}$$

수치적으로 풀면 $\beta \approx 0.2146$ rad.

수직축 각속도: $\Omega = -v_d/e = -0.5$ rad/s.

$$\mathbf{q}_d(t) = [2.5t,\ 0,\ \Omega t,\ \beta]^T$$

---

## Simulink 모델 구조

### SRR_PD.slx (논문 Fig. 5)

블록 흐름:

```
[Clock] → [Traj_gen] → ref_ddq, ref_dq, ref_q
                            ↓
ref_ddq ──────────────────→ [Sum_acc: +++]
ref_dq  → [Sum_de: +-] → [Kv] → Sum_acc
ref_q   → [Sum_e: +-]  → [Kp] → Sum_acc
                            ↓
                       [M_block] → [Sum_u: ++] → u
                                        ↑
                       [V_block] ────────┘
                            ↓
                     [System_ddq] → [Integrator_dq] → dq → [ToWS_dq]
                                          ↓
                                   [Integrator_q]  → q  → [ToWS_q]
                                          ↓
                            피드백 (q, dq → Sum_e, Sum_de, M_block, V_block, System_ddq)
```

각 MATLAB Function 블록의 역할:

| 블록 | 입력 | 출력 | 역할 |
|---|---|---|---|
| `Traj_gen` | $t$ | `ref_ddq`, `ref_dq`, `ref_q` | 목표 궤적 생성 |
| `M_block` | `acc_in`, `q` | `u_out` | $M(\mathbf{q}) \cdot \text{acc\_in}$ |
| `V_block` | `q`, `dq` | `v_out` | $V(\mathbf{q}, \dot{\mathbf{q}})$ |
| `System_ddq` | `u`, `q`, `dq` | `ddq` | $M^{-1}(\mathbf{u}-V)$ |

적분기 두 개의 역할:
- `Integrator_dq`: $\ddot{\mathbf{q}} \to \dot{\mathbf{q}}$ (각가속도 → 각속도)
- `Integrator_q`: $\dot{\mathbf{q}} \to \mathbf{q}$ (각속도 → 각도)

### SRR_PDFC.slx (논문 Fig. 6)

`SRR_PD.slx`에서 `Kp`, `Kv` Gain 블록 두 개를 `Fuzzy_gains` MATLAB Function 블록 하나로 교체한 버전.

`Fuzzy_gains` 블록:
- 입력: 위치 오차 $\mathbf{e}$, 속도 오차 $\dot{\mathbf{e}}$
- 출력: $K_p\mathbf{e}$, $K_v\dot{\mathbf{e}}$ (퍼지 이득 적용)
- 삼각형 멤버십 함수, 7×7 룰 베이스, centroid defuzzification

### Traj_gen 블록 전환

직선 궤적:
```matlab
function [ref_ddq, ref_dq, ref_q] = Traj_gen(t)
[ref_ddq, ref_dq, ref_q] = traj_linear(t);
end
```

원형 궤적:
```matlab
function [ref_ddq, ref_dq, ref_q] = Traj_gen(t)
[ref_ddq, ref_dq, ref_q] = traj_circular(t);
end
```

---

## 시뮬레이션 스크립트

### run_linear.m

논문 Fig.8, 9 재현. Kp 변화 (Kv=0.8 고정)와 Kv 변화 (Kp=1.0 고정) 조건으로 각각 3회 시뮬레이션.

`sim()` 함수로 Simulink 모델을 반복 호출하면서 `assignin('base', 'Kp', Kp)`로 workspace 변수를 바꾸는 방식.

```matlab
simOut = sim('SRR_PD', 'StopTime', '20');
results_Kp(i).dq = simOut.dq_out;
```

### run_curv_PD.m

논문 Fig.14-16 재현. 샘플링 주기 3가지 (0.001s, 0.1s, 0.15s).

`set_param(mdl, 'FixedStep', num2str(dt))`로 샘플링 주기 변경.

**주의**: `simOut.dq_out`의 형태가 열벡터로 반환될 수 있어서 reshape 처리 필요:
```matlab
N  = length(simOut.tout);
dq = reshape(simOut.dq_out, 4, N)';  % N×4 형태로 변환
```

---

## 시뮬레이션 파라미터 설정

| 설정 | 값 | 설명 |
|---|---|---|
| Solver | ode4 (Runge-Kutta) | 고정 스텝 4차 룽게쿠타 |
| 기본 스텝 크기 | 0.001 s | 논문 샘플링 주기 |
| 시뮬레이션 시간 | 20 s | |
| $K_p$ | 1.0 | 비례 이득 |
| $K_v$ | 1.0 | 미분 이득 |
| $u_{max}$ | 2.5 Nm | 최대 입력 토크 (포화) |

---

## 결과 요약

### 직선 궤적

- **PD**: $K_p$가 클수록 오버슈트 증가, 약 10초 내 0.5 m/s로 수렴
- **PDFC**: PD보다 오버슈트 작고 정착 빠름
- 정착 시간: PD ≈ 1.17s, PDFC ≈ 0.90s

### 곡선 궤적

- **PD**: 반지름 1m 원을 Reference와 거의 완벽하게 추종
- **PDFC**: 퍼지 이득 튜닝에 따라 PD와 성능 차이 발생

---

## 주의사항 및 알려진 이슈

**Traj_gen 수동 전환 필요**
`run_all.m`의 pause 구간에서 Simulink 모델을 직접 열어서 `Traj_gen` 블록 코드를 수동으로 바꿔야 한다. 저장(Ctrl+S) 후 Enter를 눌러야 변경사항이 반영된다.

**To Workspace 저장 형태**
Simulink의 To Workspace 블록 설정에서 **2차원 배열(첫 번째 차원을 따라 결합)** 으로 설정해야 한다. 그렇지 않으면 데이터가 3차원 배열로 저장되어 reshape 처리가 복잡해진다.

**Kp, Kv 초기화 경고**
Simulink 모델을 처음 열 때 workspace에 Kp, Kv가 없으면 경고가 뜬다. `run_all.m`을 통해 실행하면 자동으로 처리된다. 단독 실행 시에는 Command Window에서 `Kp=1; Kv=1;`을 먼저 입력하거나, 모델 InitFcn 콜백에 설정해둬야 한다.

**PDFC 곡선 궤적 성능**
현재 퍼지 이득 스케일 설정으로는 곡선 궤적에서 PDFC가 PD보다 오차가 크게 나온다. 퍼지 룰 베이스 및 이득 스케일 튜닝으로 개선 가능하다.

---

## 참고 파일

| 논문 Figure | 대응 스크립트 |
|---|---|
| Fig.5 (PD 블록 다이어그램) | `models/SRR_PD.slx` |
| Fig.6 (PDFC 블록 다이어그램) | `models/SRR_PDFC.slx` |
| Fig.8 (Kp 변화) | `run_linear.m` + `plot_results.m` |
| Fig.9 (Kv 변화) | `run_linear.m` + `plot_results.m` |
| Fig.12 (PD vs PDFC) | `run_linear_PDFC.m` + `plot_linear_comparison.m` |
| Fig.14-16 (곡선 PD) | `run_curv_PD.m` + `plot_curvilinear.m` |
| Fig.17-19 (곡선 PDFC) | `run_curv_PDFC.m` + `plot_curvilinear_comparison.m` |
