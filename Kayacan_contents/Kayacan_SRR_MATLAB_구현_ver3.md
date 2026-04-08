# Kayacan et al. (2012) MATLAB/Simulink 구현 가이드

> **논문**: Kayacan et al. (2012), *"Modeling and control of a spherical rolling robot: a decoupled dynamics approach"*
> **목적**: 이 문서를 처음부터 끝까지 따라하면 논문의 시뮬레이션을 그대로 재현할 수 있다.

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [사전 지식](#2-사전-지식)
3. [폴더 구조](#3-폴더-구조)
4. [MATLAB 코어 함수 구현](#4-matlab-코어-함수-구현)
5. [Simulink 모델 구축 - SRR_PD.slx](#5-simulink-모델-구축---srr_pdslx)
6. [Simulink 모델 구축 - SRR_PDFC.slx](#6-simulink-모델-구축---srr_pdfcslx)
7. [시뮬레이션 실행 및 플롯 스크립트](#7-시뮬레이션-실행-및-플롯-스크립트)
8. [전체 실행 방법](#8-전체-실행-방법)
9. [결과 해석](#9-결과-해석)
10. [주의사항 및 트러블슈팅](#10-주의사항-및-트러블슈팅)

---

## 1. 프로젝트 개요

### 무엇을 구현하는가

구형 롤링 로봇(Spherical Rolling Robot, SRR)은 구 껍데기 안에 진자(pendulum)가 달려 있어서, 진자가 기울어지면 무게중심이 이동하면서 구가 굴러가는 구조다.

이 프로젝트는 Kayacan et al. (2012) 논문의 시뮬레이션을 MATLAB/Simulink로 재현한다. 구체적으로:

- **PD 제어기**: 고정된 비례($K_p$), 미분($K_v$) 이득으로 목표 궤적 추종
- **PD-Fuzzy 제어기 (PDFC)**: 퍼지 로직으로 이득을 실시간 조정
- **직선 궤적**: 0.5 m/s 등속도 주행
- **곡선 궤적**: 반지름 1 m 원형 주행

### 논문의 핵심 아이디어: 피드백 선형화

구형 로봇의 운동방정식은 비선형이다:

$$M(\mathbf{q})\ddot{\mathbf{q}} + V(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{u}$$

여기서 $M(\mathbf{q})$는 상태에 따라 변하는 비선형 관성 행렬이고, $V$는 원심력·코리올리·중력 항이다. 이 비선형성을 제어 입력으로 직접 상쇄하는 방법이 **피드백 선형화**다.

제어 입력을:

$$\mathbf{u} = V + M\ddot{\mathbf{q}}_d + K_v\dot{\mathbf{e}} + K_p\mathbf{e}$$

로 설계하면 운동방정식에 대입했을 때 $V$와 $M$이 상쇄되어:

$$\ddot{\mathbf{e}} + K_v\dot{\mathbf{e}} + K_p\mathbf{e} = 0$$

비선형 시스템이 선형 2차 시스템처럼 동작한다.

---

## 2. 사전 지식

### 일반좌표 정의

이 시스템의 상태는 4개의 각도로 표현된다:

$$\mathbf{q} = [\theta,\ \alpha,\ \phi,\ \beta]^T$$

| 변수 | 의미 | 비고 |
|---|---|---|
| $\theta$ | x축 기준 구 회전각 | x방향 주행에 해당 |
| $\alpha$ | x축 기준 진자 회전각 | x방향 진자 기울기 |
| $\phi$ | y축 기준 구 회전각 | y방향 주행에 해당 |
| $\beta$ | y축 기준 진자 회전각 | y방향 진자 기울기 |

x-subsystem($\theta$, $\alpha$)과 y-subsystem($\phi$, $\beta$)은 논문의 디커플링 가정에 의해 독립적으로 동작한다.

### MATLAB 기초

이 프로젝트에서 사용하는 MATLAB 문법:

```matlab
% 함수 정의
function 출력변수 = 함수이름(입력변수)
    ...
end

% 구조체
p.Ms = 3;        % p라는 구조체에 Ms 필드 추가
p.mp = 2;

% 역슬래시 연산자 (선형방정식 풀기)
x = A \ b;       % inv(A)*b 와 같은 의미이지만 수치적으로 더 안정적
```

---

## 3. 폴더 구조

프로젝트 루트 폴더 이름은 `Kayacan_SRR_matsim`이다. 아래 구조대로 폴더를 만든다.

```
Kayacan_SRR_matsim/          ← MATLAB Current Folder로 설정할 위치
│
├── run_all.m                ← 마스터 실행 스크립트 (이것만 실행하면 됨)
├── main_sim.m               ← ode45 기반 단독 테스트용 스크립트
├── Kayacan_SRR_MATLAB_구현.md  ← 이 문서
│
├── core/                    ← 수식 구현 함수들
│   ├── params.m
│   ├── get_params.m
│   ├── calc_M.m
│   ├── calc_V.m
│   ├── control_input.m
│   ├── dynamics.m
│   ├── traj_linear.m
│   └── traj_circular.m
│
├── models/                  ← Simulink 모델 파일
│   ├── SRR_PD.slx
│   └── SRR_PDFC.slx
│
├── simulations/             ← 시뮬레이션 실행 스크립트
│   ├── run_linear.m
│   ├── run_linear_PDFC.m
│   ├── run_curv_PD.m
│   └── run_curv_PDFC.m
│
├── plots/                   ← 결과 플롯 스크립트
│   ├── plot_results.m
│   ├── plot_linear_comparison.m
│   ├── plot_curvilinear.m
│   └── plot_curvilinear_comparison.m
│
└── results/                 ← 시뮬레이션 결과 (.mat 파일 자동 저장)
```

---

## 4. MATLAB 코어 함수 구현

`core/` 폴더에 아래 파일들을 순서대로 만든다.

### 4.1 `params.m` — 시스템 파라미터

스크립트 파일이다. 실행하면 workspace에 변수들이 올라온다.

```matlab
% params.m
% 논문 Section 5 수치값

Ms = 3;          % 구의 질량 [kg]
mp = 2;          % 진자의 질량 [kg]
R  = 0.2;        % 구의 반지름 [m]
l  = 0.075;      % 진자의 길이 [m]
g  = 9.81;       % 중력가속도 [m/s^2]

% 관성 모멘트
Is = (2/3) * Ms * R^2;   % 구: hollow spherical shell
Ip = (1/3) * mp * l^2;   % 진자: 평행축 정리 적용

% 제어 파라미터
Kp    = 1;
Kv    = 1;
u_max = 2.5;     % 최대 입력 토크 [Nm]
```

> **Is 계산 근거**: 구는 속이 빈 껍데기(hollow spherical shell)이므로 $I^s = \frac{2}{3}M_sR^2$. 속이 꽉 찬 구(solid sphere)는 $\frac{2}{5}MR^2$이다.
>
> **Ip 계산 근거**: 진자를 균일한 가는 막대(slender rod)로 모델링. 질량 중심 기준 $\frac{1}{12}m_pl^2$에 평행축 정리를 적용하면 $I^p = \frac{1}{12}m_pl^2 + m_p(\frac{l}{2})^2 = \frac{1}{3}m_pl^2$.

---

### 4.2 `get_params.m` — Simulink용 파라미터 함수

Simulink의 MATLAB Function 블록은 workspace 변수를 직접 읽지 못한다. 이 함수를 통해 파라미터를 구조체로 반환한다.

```matlab
function p = get_params()
% Simulink MATLAB Function 블록 내부에서 호출하는 파라미터 함수

p.Ms    = 3;
p.mp    = 2;
p.R     = 0.2;
p.l     = 0.075;
p.g     = 9.81;
p.Is    = (2/3) * 3 * 0.2^2;
p.Ip    = (1/3) * 2 * 0.075^2;
p.Kp    = 1;
p.Kv    = 1;
p.u_max = 2.5;
end
```

---

### 4.3 `calc_M.m` — 관성 행렬

논문 식 (31), (32). 디커플링 가정에 의해 블록 대각 구조다.

```matlab
function M = calc_M(q, params)
% 관성 행렬 M(q) 계산
% 입력: q = [theta; alpha; phi; beta] (4x1)
%       params = 파라미터 구조체
% 출력: M = 4x4 관성 행렬

Ms = params.Ms; mp = params.mp;
R  = params.R;  l  = params.l;
Is = params.Is; Ip = params.Ip;

q1 = q(1); q2 = q(2);  % theta, alpha
q3 = q(3); q4 = q(4);  % phi, beta

% x-subsystem (theta, alpha)
M11 = Ms*R^2 + mp*R^2 + mp*l^2 + Is + Ip + 2*mp*R*l*cos(q2 - q1);
M12 = -mp*l^2 - Ip - mp*R*l*cos(q2 - q1);
M22 = mp*l^2 + Ip;

% y-subsystem (phi, beta)
M33 = Ms*R^2 + mp*R^2 + mp*l^2 + Is + Ip + 2*mp*R*l*cos(q4 - q3);
M34 = -mp*l^2 - Ip - mp*R*l*cos(q4 - q3);
M44 = mp*l^2 + Ip;

% 블록 대각 행렬 조립
M = [M11, M12,   0,   0;
     M12, M22,   0,   0;
       0,   0, M33, M34;
       0,   0, M34, M44];
end
```

---

### 4.4 `calc_V.m` — 비선형 항 벡터

논문 식 (31), (32). 원심력, 코리올리, 중력 항을 포함한다.

```matlab
function V = calc_V(q, dq, params)
% 비선형 항 V(q, dq) 계산
% 입력: q  = [theta; alpha; phi; beta] (4x1)
%       dq = [dtheta; dalpha; dphi; dbeta] (4x1)
%       params = 파라미터 구조체
% 출력: V = 4x1 벡터

mp = params.mp; R = params.R;
l  = params.l;  g = params.g;

q1 = q(1); q2 = q(2);
q3 = q(3); q4 = q(4);
dq1 = dq(1); dq2 = dq(2);
dq3 = dq(3); dq4 = dq(4);

% x-subsystem
s21 = sin(q2 - q1);   % sin(alpha - theta)
V1 = mp*R*l*s21*dq1^2 + mp*R*l*s21*dq2^2 ...
   - 2*mp*R*l*s21*dq1*dq2 - mp*g*l*s21;   % 구(theta) 방정식
V2 = mp*g*l*s21;                            % 진자(alpha) 방정식

% y-subsystem
s43 = sin(q4 - q3);   % sin(beta - phi)
V3 = mp*R*l*s43*dq3^2 + mp*R*l*s43*dq4^2 ...
   - 2*mp*R*l*s43*dq3*dq4 - mp*g*l*s43;
V4 = mp*g*l*s43;

V = [V1; V2; V3; V4];
end
```

---

### 4.5 `control_input.m` — 피드백 선형화 제어 입력

ode45 기반 `main_sim.m`에서 사용하는 함수다. Simulink 모델에서는 직접 사용하지 않는다.

```matlab
function u = control_input(q, dq, q_d, dq_d, ddq_d, params)
% 피드백 선형화 제어 입력 계산 (논문 식 41)
% u = V + Kv*de + Kp*e + M*ddq_d

Kp = params.Kp; Kv = params.Kv; u_max = params.u_max;

e  = q_d  - q;
de = dq_d - dq;

M = calc_M(q, params);
V = calc_V(q, dq, params);

u = V + Kv*de + Kp*e + M*ddq_d;
u = max(-u_max, min(u_max, u));   % 토크 포화
end
```

---

### 4.6 `dynamics.m` — 상태방정식 (ode45용)

```matlab
function dstate = dynamics(t, state, params, traj_func)
% ode45에 넘길 상태방정식
% state = [q(4x1); dq(4x1)] = 8x1 벡터

q  = state(1:4);
dq = state(5:8);

traj  = traj_func(t);
q_d   = traj(1:4);
dq_d  = traj(5:8);
ddq_d = traj(9:12);

M   = calc_M(q, params);
V   = calc_V(q, dq, params);
u   = control_input(q, dq, q_d, dq_d, ddq_d, params);
ddq = M \ (u - V);

dstate = [dq; ddq];
end
```

---

### 4.7 `traj_linear.m` — 직선 궤적

```matlab
function [ref_ddq, ref_dq, ref_q] = traj_linear(t)
% 직선 궤적: x방향 0.5 m/s 등속도 주행
v_d      = 0.5;
R        = 0.2;
dtheta_d = v_d / R;   % = 2.5 rad/s

ref_q   = [dtheta_d * t; 0; 0; 0];
ref_dq  = [dtheta_d; 0; 0; 0];
ref_ddq = [0; 0; 0; 0];
end
```

---

### 4.8 `traj_circular.m` — 원형 궤적

```matlab
function [ref_ddq, ref_dq, ref_q] = traj_circular(t)
% 원형 궤적: 곡률 반경 1 m, x방향 0.5 m/s

R  = 0.2; Ms = 3; mp = 2;
Is = (2/3)*Ms*R^2; l = 0.075; g = 9.81;
v_d = 0.5; dtheta_d = v_d / R;
e_target = 1.0;   % 목표 곡률 반경 [m]

% 논문 식 (39)에서 beta 역산 (fzero로 수치해)
% e = R*dtheta^2*(Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) / (mp*g*l*sin(beta))
beta = fzero(@(b) ...
    R*dtheta_d^2*(Is - mp*R*l*cos(b) + R^2*(Ms+mp)) ...
    / (mp*g*l*sin(b) + 1e-10) - e_target, 0.3);
% 결과: beta ≈ 0.2146 rad (약 12.3도)

Omega = -v_d / e_target;   % 수직축 각속도 = -0.5 rad/s

% phi 목표값: 구르기 조건 v_y = R*phi_dot과 원운동 기하학 v_y = v_d*sin(Omega*t)로부터
%   phi_dot_d  = (v_d/R) * sin(Omega*t)
%   phi_d      = (v_d/(R*Omega)) * (1 - cos(Omega*t))   [Omega<0이므로 음수 방향]
%   phi_ddot_d = (v_d/R) * Omega * cos(Omega*t)
phi_d      = (v_d / (R * Omega)) * (1 - cos(Omega * t));
phi_dot_d  = (v_d / R) * sin(Omega * t);
phi_ddot_d = (v_d / R) * Omega * cos(Omega * t);

ref_q   = [dtheta_d*t; 0; phi_d; beta];
ref_dq  = [dtheta_d; 0; phi_dot_d; 0];
ref_ddq = [0; 0; phi_ddot_d; 0];
end
```

> **구현 시 주의**: 기존 구현에서는 `phi_d = Omega*t` 로 잘못 설정했었다.
> 이는 구르기 조건 $v_y = R\dot{\phi}$을 무시한 것으로,
> 올바른 식은 $\dot{\phi}_d = \dfrac{v_d}{R}\sin(\Omega t)$이다.

---

## 5. Simulink 모델 구축 — SRR_PD.slx

논문 Fig. 5의 PD 제어기 블록 다이어그램을 구현한다.

### 5.1 새 모델 만들기

MATLAB Command Window에서:

```matlab
simulink
```

Simulink Start Page → **Blank Model** → 파일명 `SRR_PD`로 `models/` 폴더에 저장.

### 5.2 솔버 설정

**Ctrl+E** (구성 파라미터 창 열기):

| 항목 | 설정값 |
|---|---|
| 시작 시간 | `0.0` |
| 중지 시간 | `20.0` |
| 유형 | **고정 스텝** |
| 솔버 | **ode4 (Runge-Kutta)** |
| 고정 스텝 크기 | `0.001` |

> **고정 스텝 선택 이유**: 가변 스텝 솔버(ode45 등)는 목록에 있지만 고정 스텝 솔버(ode4)는 유형을 "고정 스텝"으로 먼저 바꿔야 나타난다.

### 5.3 InitFcn 콜백 설정

Simulink 모델을 처음 열 때 workspace에 `Kp`, `Kv`가 없으면 Gain 블록에 경고가 뜬다. 이를 방지하기 위해 모델 초기화 콜백을 설정한다.

**Modeling 탭 → Model Properties → Callbacks → InitFcn**에 입력:

```matlab
Kp = 1;
Kv = 1;
```

### 5.4 블록 배치

캔버스 빈 곳을 **더블클릭**하면 블록 검색창이 열린다. 블록 이름을 검색해서 추가한다.

#### 블록 목록 및 설정

| 블록 이름 | 검색어 | 설정 |
|---|---|---|
| `Clock` | `Clock` | 기본값 유지 |
| `Traj_gen` | `MATLAB Function` | 아래 코드 입력 |
| `Sum_acc` | `Sum` | List of signs: `+++` |
| `Kp` | `Gain` | Gain: `Kp` |
| `Kv` | `Gain` | Gain: `Kv` |
| `Sum_de` | `Sum` | List of signs: `+-` |
| `Sum_e` | `Sum` | List of signs: `+-` |
| `M_block` | `MATLAB Function` | 아래 코드 입력 |
| `V_block` | `MATLAB Function` | 아래 코드 입력 |
| `Sum_u` | `Sum` | List of signs: `++` |
| `System_ddq` | `MATLAB Function` | 아래 코드 입력 |
| `Integrator_dq` | `Integrator` | Initial condition: `zeros(4,1)` |
| `Integrator_q` | `Integrator` | Initial condition: `zeros(4,1)` |
| `ToWS_dq` | `To Workspace` | Variable name: `dq_out`, 저장 형식: 배열, 2차원 신호: 첫 번째 차원을 따라 결합 |
| `ToWS_q` | `To Workspace` | Variable name: `q_out`, 저장 형식: 배열, 2차원 신호: 첫 번째 차원을 따라 결합 |

> **To Workspace 설정 중요**: "2차원 신호를 다음으로 저장" 옵션을 반드시 **2차원 배열(첫 번째 차원을 따라 결합)**으로 설정해야 한다. 기본값인 "3차원 배열(세 번째 차원을 따라 결합)"으로 두면 데이터 shape이 달라져서 플롯 스크립트에서 에러가 난다.

### 5.5 MATLAB Function 블록 코드 입력

각 블록을 더블클릭하면 코드 에디터가 열린다. 기존 코드를 지우고 아래 코드를 붙여넣는다.

#### `Traj_gen` 블록

```matlab
function [ref_ddq, ref_dq, ref_q] = Traj_gen(t)
[ref_ddq, ref_dq, ref_q] = traj_linear(t);
end
```

> 직선 궤적 시뮬레이션 시 `traj_linear`, 원형 궤적 시뮬레이션 시 `traj_circular`로 바꾼다.

#### `M_block` 블록

```matlab
function u_out = M_block(acc_in, q)
% M(q) * acc_in
p = get_params();
M = calc_M(q, p);
u_out = M * acc_in;
end
```

#### `V_block` 블록

```matlab
function v_out = V_block(q, dq)
% V(q, dq) 비선형 항
p = get_params();
v_out = calc_V(q, dq, p);
end
```

#### `System_ddq` 블록

```matlab
function ddq = System_ddq(u, q, dq)
% ddq = M^{-1} * (u - V)
p = get_params();
M = calc_M(q, p);
V = calc_V(q, dq, p);
ddq = M \ (u - V);
end
```

### 5.6 신호 연결

블록 출력 포트에서 마우스를 드래그해서 입력 포트로 연결한다. 신호를 분기할 때는 연결된 선 위에서 **Ctrl+클릭** 후 드래그하면 된다.

#### 연결 목록

```
Clock          → Traj_gen (입력: t)

Traj_gen (ref_ddq) → Sum_acc (입력 1번, +)
Traj_gen (ref_dq)  → Sum_de (입력 1번, +)
Traj_gen (ref_q) → Sum_e (입력 1번, +)

Sum_de → Kv → Sum_acc (입력 2번)
Sum_e  → Kp → Sum_acc (입력 3번)

Sum_acc → M_block (acc_in)

q 피드백선 → M_block (q)
q 피드백선 → V_block (q)
q 피드백선 → System_ddq (q)
q 피드백선 → Sum_e (입력 2번, -)

dq 피드백선 → V_block (dq)
dq 피드백선 → System_ddq (dq)
dq 피드백선 → Sum_de (입력 2번, -)

M_block (u_out) → Sum_u (입력 1번, +)
V_block (v_out) → Sum_u (입력 2번, +)

Sum_u → System_ddq (u)

System_ddq (ddq) → Integrator_dq → Integrator_q
Integrator_dq 출력 = dq  → ToWS_dq, 피드백으로 분기
Integrator_q  출력 = q   → ToWS_q, 피드백으로 분기
```

> **ref_q 생성 방법**: ref_q는 Traj_gen 블록에서 직접 출력된다. traj_linear.m, traj_circular.m 내부에서 ref_dq를 시간에 대해 적분한 수식으로 ref_q를 계산하여 반환한다.

### 5.7 완성된 블록 다이어그램 구조


![[Pasted image 20260407235331.png]]


### 5.8 모델 저장

**Ctrl+S**. `models/SRR_PD.slx`로 저장.

---

## 6. Simulink 모델 구축 — SRR_PDFC.slx

`SRR_PD.slx`를 기반으로 Fuzzy 제어기를 추가한다.

### 6.1 파일 복사

Windows 탐색기에서 `models/SRR_PD.slx`를 복사해서 같은 폴더에 `SRR_PDFC.slx`로 붙여넣기.

### 6.2 Kp, Kv Gain 블록 교체

`SRR_PDFC.slx`를 열고:

1. `Kp` Gain 블록과 `Kv` Gain 블록을 **삭제**한다.
2. 그 자리에 **MATLAB Function 블록** 하나를 추가하고 이름을 `Fuzzy_gains`로 설정한다.
3. `Fuzzy_gains` 블록을 더블클릭해서 아래 코드를 입력한다.

#### `Fuzzy_gains` 블록 코드

```matlab
function [Kp_e, Kv_de] = Fuzzy_gains(e, de)

% 4채널 PD-type Fuzzy Controller

% 입력: e (위치 오차 4x1), de (속도 오차 4x1)

% 출력: Kp*e (4x1), Kv*de (4x1)

Kp_base = 1.0;

Kv_base = 1.0;

% [Q&A 반영] 스케일링(Scaling) 배열 정의

% 왜 스케일링이 필요한가?

% 1번, 3번 성분(전진/조향 속도)은 오차가 최대 2.5까지 커져서 [-3, 3] 범위에 잘 맞지만(scale=1.0),

% 2번, 4번 성분(진자 각도)은 오차가 기껏해야 0.2 rad 정도로 매우 작습니다.

% 이 작은 오차를 5배(1/0.2) 뻥튀기해주어야 퍼지 범위[-3, 3] 안에서 유의미한 제어가 일어납니다.

scale_e_list = [1.0; 0.2; 1.0; 0.2];

scale_de_list = [1.0; 0.5; 1.0; 0.5];

% 4개 성분의 계산된 퍼지 이득을 담을 빈 배열 준비

Kp_fuzzy = zeros(4, 1);

Kv_fuzzy = zeros(4, 1);

% 1번부터 4번까지 모든 상태 변수에 대해 퍼지 로직 반복 수행

for i = 1:4

% 각 채널에 맞는 스케일링 값을 fuzzy_gain 함수로 전달합니다.

Kp_fuzzy(i) = fuzzy_gain(e(i), de(i), Kp_base, scale_e_list(i), scale_de_list(i));

% Kv를 구할 때는 룰 베이스(Table II) 기준에 맞추기 위해 e와 de의 순서를 뒤집어 넣습니다.

Kv_fuzzy(i) = fuzzy_gain(de(i), e(i), Kv_base, scale_de_list(i), scale_e_list(i));

end

% 계산된 각각의 퍼지 이득을 해당하는 오차(e, de) 배열에 요소별로 곱해줍니다. (.* 연산자)

Kp_e = Kp_fuzzy .* e;

Kv_de = Kv_fuzzy .* de;

end

function K = fuzzy_gain(err, derr, K_base, scale_e, scale_de)

% 삼각형 멤버십 함수 기반 퍼지 이득 조정

% 언어 변수 중심점 정의 (NL, NM, NS, ZR, PS, PM, PL)

centers = [-3, -2, -1, 0, 1, 2, 3];

% [Q&A 반영] 퍼지화 (Fuzzification)

% 실제 입력된 오차 숫자를 중심점 범위([-3, 3])에 맞게 스케일링한 후,

% 각 그룹에 몇 %나 속하는지 0~1 사이의 확률(소속도, mu)로 변환합니다.

mu_e = trimf_vector(err / scale_e, centers);

mu_de = trimf_vector(derr / scale_de, centers);

% 룰 베이스 (논문 Table II): 전문가의 지식을 코드로 옮긴 표

rule_base = [

0, 1, 2, 3, 3, 3, 3; % PL (행: 속도 오차 de)

-1, 0, 1, 2, 3, 3, 3; % PM

-2,-1, 0, 1, 2, 3, 3; % PS

-3,-2,-1, 0, 1, 2, 3; % ZR

-3,-3,-2,-1, 0, 1, 2; % NS

-3,-3,-3,-2,-1, 0, 1; % NM

-3,-3,-3,-3,-2,-1, 0 % NL

];

% 중심법(centroid)을 이용한 역퍼지화(Defuzzification)

numerator = 0;

denominator = 0;

for i = 1:7

for j = 1:7

% [Q&A 반영] 왜 최솟값(min)을 쓸까요?

% 퍼지 논리에서 "오차가 크고(AND) 속도 오차가 크다"라는 조건은

% 두 확률 중 더 작은 값(약점/병목)을 전체 만족도(발동 강도, firing)로 취합니다.

firing = min(mu_e(i), mu_de(j));

% 행렬의 1번 행이 수학적 PL(인덱스 7)에 해당하므로 8-i로 뒤집어 매칭

output_center = rule_base(8-i, j);

% 가중 평균을 내기 위한 분자/분모 누적

numerator = numerator + firing * output_center;

denominator = denominator + firing;

end

end

% 0으로 나누는 오류 방지

if denominator > 1e-10

delta_K = numerator / denominator;

else

delta_K = 0;

end

% [Q&A 반영] 0.5 스케일링의 근거와 안전장치(max)

% 0.5를 곱하는 이유: 퍼지가 제안한 변화폭(delta_K, -3~3)을 그대로 더하면

% 제어 이득이 4배로 뛰거나 마이너스가 되어 로봇이 미쳐 날뛰게 됩니다.

% 따라서 변화의 민감도를 부드럽게 억제하는(튜닝된) 쿠션 역할을 합니다.

% max(0.1, ...) 이유: 최종 제어 이득이 음수나 0이 되어 시스템이 불안정해지는 것을 막습니다.

K = max(0.1, K_base + 0.5 * delta_K);

end

function mu = trimf_vector(x, centers)

% 7개의 중심점(NL~PL)을 돌면서 각각의 삼각형 멤버십 함수를 호출하는 헬퍼 함수

n = length(centers);

mu = zeros(1, n);

for i = 1:n

if i == 1

% 양 끝단(NL, PL)은 밖으로 무한히 뻗어나가도록 처리

mu(i) = trimf(x, centers(1)-1, centers(1), centers(2));

elseif i == n

mu(i) = trimf(x, centers(n-1), centers(n), centers(n)+1);

else

mu(i) = trimf(x, centers(i-1), centers(i), centers(i+1));

end

end

end

function y = trimf(x, a, b, c)

% [Q&A 반영] 삼각형 멤버십 함수 (Triangular Function)

% a(왼쪽 끝), b(정상), c(오른쪽 끝)으로 이루어진 삼각형을 만들어

% 현재 값 x가 이 삼각형에 몇 %나 속하는지 0~1(0~100%)로 반환합니다.

% 컴퓨터가 계산하기 아주 가볍고 직관적이라 로봇 제어에 가장 많이 쓰입니다.

if x <= a || x >= c

y = 0; % 삼각형 범위를 아예 벗어난 경우 (0%)

elseif x <= b

y = (x - a) / (b - a); % 왼쪽 오르막길 (비례식으로 0~1 계산)

else

y = (c - x) / (c - b); % 오른쪽 내리막길 (비례식으로 1~0 계산)

end

end
```

### 6.3 신호 재연결

`Fuzzy_gains` 블록은 입력 2개, 출력 2개다.

```
Sum_e  (e  출력) → Fuzzy_gains (입력 1번: e)
Sum_de (de 출력) → Fuzzy_gains (입력 2번: de)

Fuzzy_gains (출력 1번: Kp_e)  → Sum_acc (입력 2번)
Fuzzy_gains (출력 2번: Kv_de) → Sum_acc (입력 3번)
```

나머지 연결은 `SRR_PD.slx`와 동일하다.

![[Pasted image 20260407235444.png]]

### 6.4 솔버 및 InitFcn 설정

`SRR_PD.slx`와 동일하게 설정한다.

- 솔버: ode4, 고정 스텝, 0.001 s, 중지 시간 20 s
- InitFcn: `Kp = 1; Kv = 1;`

### 6.5 모델 저장

**Ctrl+S**. `models/SRR_PDFC.slx`로 저장.

---

## 7. 시뮬레이션 실행 및 플롯 스크립트

### 7.1 `simulations/run_linear.m`

논문 Fig.8 (Kp 변화), Fig.9 (Kv 변화) 재현.

```matlab
% run_linear.m
% 논문 Fig.8, 9, 12 재현
% 여러 Kp, Kv 조건으로 시뮬레이션 반복 실행

function run_linear()
clear; clc;
addpath('core');
addpath('models');
mdl = 'SRR_PD';

% 모델 로드
load_system(mdl);

%% 케이스 1: Kv=0.8 고정, Kp 변화 (논문 Fig.8)
Kv = 0.8;
Kp_list = [0.6, 0.8, 1.0];
results_Kp = struct();

fprintf('=== Kp 변화 시뮬레이션 ===\n');
for i = 1:length(Kp_list)
    Kp = Kp_list(i);
    fprintf('Kp=%.1f, Kv=%.1f 실행 중...\n', Kp, Kv);
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setModelParameter('StopTime', '20');
    simIn = simIn.setVariable('Kp', Kp);
    simIn = simIn.setVariable('Kv', Kv);
    simOut = sim(simIn);

    results_Kp(i).Kp   = Kp;
    results_Kp(i).Kv   = Kv;
    results_Kp(i).tout = simOut.tout;
    results_Kp(i).q    = simOut.q_out;
    results_Kp(i).dq   = simOut.dq_out;
end

%% 케이스 2: Kp=1.0 고정, Kv 변화 (논문 Fig.9)
Kp = 1.0;
Kv_list = [0.6, 0.8, 1.0];
results_Kv = struct();

fprintf('\n=== Kv 변화 시뮬레이션 ===\n');
for i = 1:length(Kv_list)
    Kv = Kv_list(i);
    fprintf('Kp=%.1f, Kv=%.1f 실행 중...\n', Kp, Kv);
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setModelParameter('StopTime', '20');
    simIn = simIn.setVariable('Kp', Kp);
    simIn = simIn.setVariable('Kv', Kv);
    simOut = sim(simIn);

    results_Kv(i).Kp   = Kp;
    results_Kv(i).Kv   = Kv;
    results_Kv(i).tout = simOut.tout;
    results_Kv(i).q    = simOut.q_out;
    results_Kv(i).dq   = simOut.dq_out;
end

%% 결과 저장
save('results/results_linear.mat', 'results_Kp', 'results_Kv');
fprintf('\n결과 저장 완료: results_linear.mat\n');
end
```

> **함수로 작성한 이유**: 스크립트로 작성하면 `clear`가 호출자(`run_all.m`)의 workspace까지 초기화한다. 함수로 감싸면 `clear`가 함수 내부 workspace만 초기화하여 단독 실행과 `run_all.m` 실행 모두에서 올바르게 동작한다. 단독 실행 시에는 프로젝트 루트를 Current Folder로 설정한 뒤 `run_linear()` 명령으로 호출한다.
>
> **`Simulink.SimulationInput` 사용 이유**: `assignin` 방식은 모델 외부 workspace 상태에 의존하여 재현성이 떨어진다. `SimulationInput` 객체로 파라미터를 명시적으로 주입하면 병렬 시뮬레이션(parsim)과도 호환되고, 어느 환경에서 실행해도 동일한 결과가 보장된다.
>
> **raw 저장(reshape 없음)**: `q_out`, `dq_out`을 reshape 없이 raw 형태로 저장한다. reshape은 플롯 스크립트(`plot_results.m`)에서 일괄 처리한다.

### 7.2 `simulations/run_linear_PDFC.m`

```matlab
% run_linear_PDFC.m
% 논문 Fig.12 재현
% PD-Fuzzy 제어기, Kp=1 Kv=1 직선 궤적

function run_linear_PDFC()
clear; clc;
addpath('core');
addpath('models');

mdl = 'SRR_PDFC';
load_system(mdl);

fprintf('=== PD-Fuzzy 직선 궤적 시뮬레이션 ===\n');
fprintf('Kp=1.0, Kv=1.0 실행 중...\n');

simOut = sim(mdl, 'StopTime', '20');

results_PDFC.Kp   = 1.0;
results_PDFC.Kv   = 1.0;
results_PDFC.tout = simOut.tout;
results_PDFC.q    = simOut.q_out;
results_PDFC.dq   = simOut.dq_out;

save('results/results_linear_PDFC.mat', 'results_PDFC');
fprintf('결과 저장 완료: results_linear_PDFC.mat\n');
end
```

### 7.3 `simulations/run_curv_PD.m`

논문 Fig.14-16 재현. 샘플링 주기 3가지 비교.

```matlab
% run_curv_PD.m
% 논문 Fig.14, 15, 16 재현
% 원형 궤적 + PD 제어, 샘플링 주기 3가지

function run_curv_PD()
clear; clc;
addpath('core');
addpath('models');
mdl = 'SRR_PD';
load_system(mdl);
v_d = 0.5; e_target = 1.0;
Omega = -v_d / e_target;
T_sim = 2*pi / abs(Omega);   % 원 한 바퀴 주기 ≈ 12.57 s
step_list = [0.001, 0.5, 1.0];
results_curv_PD = struct();
fprintf('=== 곡선 궤적 PD 시뮬레이션 ===\n');

for i = 1:length(step_list)
    dt = step_list(i);
    fprintf('샘플링 주기 %.3f s 실행 중...\n', dt);
    
    % --- 수정된 부분 (안전한 파라미터 전달) ---
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setVariable('Kp', 1.0);
    simIn = simIn.setVariable('Kv', 1.0);
    simIn = simIn.setModelParameter('FixedStep', num2str(dt));
    simIn = simIn.setModelParameter('StopTime', num2str(T_sim));
    simOut = sim(simIn);
    % -----------------------------------------
    
    N  = length(simOut.tout);
    q  = reshape(simOut.q_out,  4, N)';
    dq = reshape(simOut.dq_out, 4, N)';
    results_curv_PD(i).dt   = dt;
    results_curv_PD(i).tout = simOut.tout;
    results_curv_PD(i).q    = q;
    results_curv_PD(i).dq   = dq;
end

save('results/results_curv_PD.mat', 'results_curv_PD');
fprintf('결과 저장 완료: results_curv_PD.mat\n');
end
```

> **샘플링 주기 변경**: `[0.001, 0.1, 0.15]`에서 `[0.001, 0.5, 1.0]`으로 변경하여 더 큰 샘플링 주기 차이가 궤적 추종에 미치는 영향을 확인한다.
>
> **`Simulink.SimulationInput` 사용 이유**: `set_param` + `assignin` 조합은 모델 영구 파일을 직접 수정하는 부작용이 있다. `SimulationInput` 객체를 사용하면 모델 파일은 수정되지 않으면서 시뮬레이션 실행 시에만 파라미터가 적용된다.
>
> **곡선 궤적 reshape 처리**: 곡선 궤적은 plot 스크립트에서 채널별 접근이 잦으므로 저장 시 미리 `N×4` 형태로 reshape한다.

### 7.4 `simulations/run_curv_PDFC.m`

```matlab
% run_curv_PDFC.m
% 논문 Fig.17, 18, 19 재현
% 원형 궤적 + PD-Fuzzy 제어, 샘플링 주기 3가지

function run_curv_PDFC()
clear; clc;
addpath('core');
addpath('models');

mdl = 'SRR_PDFC';
load_system(mdl);

v_d = 0.5; e_target = 1.0;
Omega = -v_d / e_target;
T_sim = 2*pi / abs(Omega);   % 원 한 바퀴 주기 ≈ 12.57 s

step_list = [0.001, 0.5, 1.0];
results_curv_PDFC = struct();

fprintf('=== 곡선 궤적 PDFC 시뮬레이션 ===\n');
for i = 1:length(step_list)
    dt = step_list(i);
    fprintf('샘플링 주기 %.3f s 실행 중...\n', dt);

    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setModelParameter('FixedStep', num2str(dt));
    simIn = simIn.setModelParameter('StopTime', num2str(T_sim));
    simOut = sim(simIn);

    N  = length(simOut.tout);
    q  = reshape(simOut.q_out,  4, N)';
    dq = reshape(simOut.dq_out, 4, N)';

    results_curv_PDFC(i).dt   = dt;
    results_curv_PDFC(i).tout = simOut.tout;
    results_curv_PDFC(i).q    = q;
    results_curv_PDFC(i).dq   = dq;
end

save('results/results_curv_PDFC.mat', 'results_curv_PDFC');
fprintf('결과 저장 완료: results_curv_PDFC.mat\n');
end
```

---

### 7.5 `plots/plot_results.m` — 직선 궤적 플롯

`run_linear.m` 실행 후 저장된 `results_linear.mat`을 로드해서 속도 추종 결과를 플롯한다. 논문 Fig.8 (Kp 변화), Fig.9 (Kv 변화) 전체 케이스 비교 플롯과, `Kp=1, Kv=1` 단독 케이스의 상세 플롯(속도, theta, 진자각 alpha, 속도 오차)을 함께 생성한다.

```matlab
% plot_results.m
% 논문 Fig.8, Fig.9 스타일 플롯
% run_linear.m 실행 후 사용

function plot_results()
clear; clc;

%% 파라미터
R   = 0.2;
v_d = 0.5;

%% 결과 로드
load('results/results_linear.mat');

%% 데이터 형태 변환 (80004x1 → 20001x4)
for i = 1:length(results_Kp)
    N = length(results_Kp(i).tout);
    results_Kp(i).dq = reshape(results_Kp(i).dq, 4, N)';
    results_Kp(i).q  = reshape(results_Kp(i).q,  4, N)';
end
for i = 1:length(results_Kv)
    N = length(results_Kv(i).tout);
    results_Kv(i).dq = reshape(results_Kv(i).dq, 4, N)';
    results_Kv(i).q  = reshape(results_Kv(i).q,  4, N)';
end

%% ============================================================
%% Fig.8 재현: Kv=0.8 고정, Kp 변화
% ============================================================
figure('Name', 'Fig.8 - Kp variation', 'Position', [100, 100, 700, 450]);
hold on;

colors = {'b', 'r', 'g'};
tout_ref = results_Kp(1).tout;
v_ref    = v_d * ones(size(tout_ref));
plot(tout_ref, v_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

for i = 1:length(results_Kp)
    dtheta   = results_Kp(i).dq(:, 1);
    v_actual = R * abs(dtheta);
    label    = sprintf('Kp=%.1f Kv=%.1f', results_Kp(i).Kp, results_Kp(i).Kv);
    plot(results_Kp(i).tout, v_actual, colors{i}, ...
        'LineWidth', 1.5, 'DisplayName', label);
end

xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Step responses for various proportional gains');
legend('Location', 'southeast'); grid on;
ylim([-0.05, 0.85]);

%% ============================================================
%% Fig.9 재현: Kp=1.0 고정, Kv 변화
%% ============================================================
figure('Name', 'Fig.9 - Kv variation', 'Position', [200, 200, 700, 450]);
hold on;

tout_ref = results_Kv(1).tout;
v_ref    = v_d * ones(size(tout_ref));
plot(tout_ref, v_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

for i = 1:length(results_Kv)
    dtheta   = results_Kv(i).dq(:, 1);
    v_actual = R * abs(dtheta);
    label    = sprintf('Kp=%.1f Kv=%.1f', results_Kv(i).Kp, results_Kv(i).Kv);
    plot(results_Kv(i).tout, v_actual, colors{i}, ...
        'LineWidth', 1.5, 'DisplayName', label);
end

xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Step responses for various derivative gains');
legend('Location', 'southeast'); grid on;
ylim([-0.05, 0.85]);

%% ============================================================
%% 추가: Kp=1, Kv=1 단독 결과 상세 플롯
%% ============================================================
idx = find([results_Kv.Kv] == 1.0, 1);
res = results_Kv(idx);

dtheta   = res.dq(:, 1);
alpha    = res.q(:, 2);
v_actual = R * abs(dtheta);
v_ref    = v_d * ones(size(res.tout));
v_error  = v_ref - v_actual;

figure('Name', 'Kp=1 Kv=1 Detail', 'Position', [300, 300, 900, 700]);

subplot(2,2,1);
plot(res.tout, v_ref,    'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference'); hold on;
plot(res.tout, v_actual, 'b-',  'LineWidth', 1.5, 'DisplayName', 'PD');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Tracking (Kp=1, Kv=1)');
legend; grid on; ylim([-0.05, 0.8]);

subplot(2,2,2);
theta_d = (v_d/R) * res.tout;
plot(res.tout, theta_d,    'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference'); hold on;
plot(res.tout, res.q(:,1), 'b-',  'LineWidth', 1.5, 'DisplayName', 'PD');
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Theta Tracking'); legend; grid on;

subplot(2,2,3);
plot(res.tout, alpha*(180/pi), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\alpha (deg)');
title('Pendulum Angle \alpha'); grid on;

subplot(2,2,4);
plot(res.tout, v_error, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Error'); grid on;

%% 결과 출력
fprintf('=== Kp=1, Kv=1 결과 요약 ===\n');
fprintf('최대 속도 오차: %.4f m/s\n', max(abs(v_error)));
idx2 = find(abs(v_error) < 0.01, 1, 'first');
if ~isempty(idx2)
    fprintf('정착 시간 (|e|<0.01): %.2f s\n', res.tout(idx2));
end
end
```

> **reshape 처리 이유**: `run_linear.m`에서는 raw 데이터를 그대로 저장한다. 플롯 스크립트에서 `reshape(data, 4, N)'`으로 `N×4` 행렬로 변환해야 `dq(:,1)` 형태로 각 채널을 뽑을 수 있다.
>
> **생성되는 figure**: Fig.8 (Kp 변화 비교), Fig.9 (Kv 변화 비교), Kp=1 Kv=1 상세 플롯 — 총 3개.

---

### 7.6 `plots/plot_linear_comparison.m` — PD vs PDFC 직선 궤적 비교

논문 Fig.12 재현. `results_linear.mat`과 `results_linear_PDFC.mat` 두 파일을 로드해서 속도 추종 및 오차를 비교하고, 각각의 정착 시간도 출력한다.

```matlab
% plot_linear_comparison.m
% 논문 Fig.12 재현: PD vs PD-Fuzzy 비교

function plot_linear_comparison()
clear; clc;
addpath('results');

R   = 0.2;
v_d = 0.5;

%% 결과 로드
load('results/results_linear.mat');
load('results/results_linear_PDFC.mat');

%% Kp=1, Kv=1 PD 결과 추출
for i = 1:length(results_Kv)
    N = length(results_Kv(i).tout);
    results_Kv(i).dq = reshape(results_Kv(i).dq, 4, N)';
    results_Kv(i).q  = reshape(results_Kv(i).q,  4, N)';
end
idx = find([results_Kv.Kv] == 1.0);
res_PD = results_Kv(idx);

%% PDFC 결과 추출
N = length(results_PDFC.tout);
results_PDFC.dq = reshape(results_PDFC.dq, 4, N)';
results_PDFC.q  = reshape(results_PDFC.q,  4, N)';
res_PDFC = results_PDFC;

%% 속도 계산
v_PD   = R * abs(res_PD.dq(:,1));
v_PDFC = R * abs(res_PDFC.dq(:,1));
v_ref  = v_d * ones(size(res_PD.tout));

e_PD   = v_ref - v_PD;
e_PDFC = v_d * ones(size(res_PDFC.tout)) - v_PDFC;

%% Fig.12 스타일 플롯: 속도 추종 비교
figure('Name', 'Fig.12 - PD vs PDFC', 'Position', [100, 100, 700, 450]);
hold on;
plot(res_PD.tout,   v_ref,  'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(res_PD.tout,   v_PD,   'b-',  'LineWidth', 1.5, 'DisplayName', 'PD');
plot(res_PDFC.tout, v_PDFC, 'r-',  'LineWidth', 1.5, 'DisplayName', 'PDFC');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Step responses with PD and PD-Fuzzy controller');
legend('Location', 'southeast'); grid on;
ylim([-0.05, 0.85]);

%% 속도 오차 비교
figure('Name', 'Velocity Error Comparison', 'Position', [200, 200, 700, 450]);
hold on;
plot(res_PD.tout,   e_PD,   'b-', 'LineWidth', 1.5, 'DisplayName', 'PD');
plot(res_PDFC.tout, e_PDFC, 'r-', 'LineWidth', 1.5, 'DisplayName', 'PDFC');
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Error: PD vs PDFC');
legend; grid on;

%% 결과 출력
fprintf('=== PD vs PDFC 비교 ===\n');
fprintf('PD   최대 오차: %.4f m/s\n', max(abs(e_PD)));
fprintf('PDFC 최대 오차: %.4f m/s\n', max(abs(e_PDFC)));

idx_PD = find(abs(e_PD) < 0.01, 1, 'first');
idx_FC = find(abs(e_PDFC) < 0.01, 1, 'first');
if ~isempty(idx_PD)
    fprintf('PD   정착 시간: %.2f s\n', res_PD.tout(idx_PD));
end
if ~isempty(idx_FC)
    fprintf('PDFC 정착 시간: %.2f s\n', res_PDFC.tout(idx_FC));
end
end
```

---

### 7.7 `plots/plot_curvilinear.m` — PD 원형 궤적 플롯

논문 Fig.14-16 재현. `results_curv_PD.mat`을 로드해서 샘플링 주기 3가지 케이스의 XY 궤적을 그린다.

#### XY 위치 복원 방법

`phi`(y축 기준 구 회전각)는 heading angle이 아니므로 직접 XY 적분에 쓸 수 없다. 대신 논문 식 (33) `Ω = -Rθ̇/e`를 이용해 heading angle `psi`를 복원한 뒤, 실제 beta에서 식 (39)로 `e_actual`을 매 시점 계산해서 사용한다.

```
psi_dot = -R * dtheta / e_actual   (식 33)
dot_x = R * dtheta * sin(psi)
dot_y = -R * dtheta * cos(psi)
```

`e_actual`을 고정값 `e_target`으로 쓰면 beta 오차가 얼마이든 heading이 reference와 동일해져서 실제 궤적과 reference가 항상 겹쳐 보이는 버그가 생긴다.

```matlab
% plot_curvilinear.m
% 논문 Fig.14-16 재현: 원형 궤적 추종 결과

function plot_curvilinear()
clear; clc;
addpath('results');
addpath('core');

R        = 0.2;
v_d      = 0.5;
e_target = 1.0;
Omega    = -v_d / e_target;   % = -0.5 rad/s
T_sim    = 2*pi / abs(Omega); % 원 한 바퀴 주기 ≈ 12.57 s

% 식 (39) 계산에 필요한 파라미터
Ms = 3; mp = 2; l = 0.075; g = 9.81;
Is = (2/3) * Ms * R^2;

%% 목표 원형 궤적 (기하학적 원 공식, phi에 무관)
% 곡률 반경 e_target의 원: 중심 (e_target*(cos(0)-1), 0) = (-e,0) → 반지름 e
t_ref = linspace(0, T_sim, 2000);
x_ref = e_target * (cos(Omega * t_ref) - 1);
y_ref = e_target *  sin(Omega * t_ref);

%% 결과 로드
load('results/results_curv_PD.mat');

%% 플롯
figure('Name', 'Fig.14-16 Circular Trajectory PD', ...
       'Position', [100, 100, 1100, 400]);

titles = {'dt = 0.001 s', 'dt = 0.5 s', 'dt = 1.0 s'};

for i = 1:3
    res  = results_curv_PD(i);
    tout = res.tout;

    dtheta = res.dq(:,1);
    beta   = res.q(:,4);

    % 실제 곡률 반경 e_actual: 식 (39) [매 시점 beta에서 계산]
    % e = R*dtheta^2*(Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) / (mp*g*l*sin(beta))
    e_actual = R .* dtheta.^2 .* (Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) ...
               ./ (mp*g*l*sin(beta) + 1e-10);
    % 수치 발산 방지 (과도 구간 등 beta≈0일 때 e가 비정상적으로 커짐)
    e_actual = max(min(e_actual, 10*e_target), 0.1*e_target);

    % heading angle psi: psi_dot = -R*dtheta/e_actual (식 33)
    psi = cumtrapz(tout, -R * dtheta ./ e_actual);

    % 글로벌 XY 적분
    dx = R * dtheta .* sin(psi);
    dy = -R * dtheta .* cos(psi);

    x = cumtrapz(tout, dx);
    y = cumtrapz(tout, dy);

    subplot(1, 3, i);
    plot(x_ref, y_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference'); hold on;
    plot(x, y,         'b-',  'LineWidth', 1, 'DisplayName', 'Actual');
    xlabel('X (m)'); ylabel('Y (m)');
    title(titles{i});
    legend('Location', 'best'); grid on;
    axis equal;
end

sgtitle('Circular Trajectory Tracking - PD Controller');
end
```

---

### 7.8 `plots/plot_curvilinear_comparison.m` — PD vs PDFC 원형 궤적 비교

논문 Fig.14-19 재현. PD와 PDFC 결과를 각각 별도 figure로 표시하고, phi 추종 오차를 수치로 출력한다.

phi 추종 오차의 reference는 구르기 조건으로부터 유도된 올바른 목표값이다:

$$\phi_d(t) = \frac{v_d}{R\Omega}(1 - \cos(\Omega t))$$

```matlab
% plot_curvilinear_comparison.m
% 논문 Fig.14-19 재현: PD vs PDFC 원형 궤적 비교

function plot_curvilinear_comparison()
clear; clc;
addpath('results');

R        = 0.2;
v_d      = 0.5;
e_target = 1.0;
Omega    = -v_d / e_target;   % = -0.5 rad/s
T_sim    = 2*pi / abs(Omega); % 원 한 바퀴 주기 ≈ 12.57 s

% 식 (39) 계산에 필요한 파라미터
Ms = 3; mp = 2; l = 0.075; g = 9.81;
Is = (2/3) * Ms * R^2;

%% 목표 원형 궤적 (기하학적 원 공식, phi에 무관)
t_ref = linspace(0, T_sim, 2000);
x_ref = e_target * (cos(Omega * t_ref) - 1);
y_ref = e_target *  sin(Omega * t_ref);

%% 결과 로드
load('results/results_curv_PD.mat');
load('results/results_curv_PDFC.mat');

titles = {'dt = 0.001 s', 'dt = 0.5 s', 'dt = 1.0 s'};

%% PD 결과 플롯 (Fig.14-16)
figure('Name', 'Fig.14-16 PD', 'Position', [100, 100, 1100, 400]);
for i = 1:3
    res    = results_curv_PD(i);
    tout   = res.tout;
    dtheta = res.dq(:,1);
    beta   = res.q(:,4);

    e_actual = R .* dtheta.^2 .* (Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) ...
               ./ (mp*g*l*sin(beta) + 1e-10);
    e_actual = max(min(e_actual, 10000*e_target), 0.01*e_target);
    psi = cumtrapz(tout, -R * dtheta ./ e_actual);
    dx  = R * dtheta .* sin(psi);
    dy  = -R * dtheta .* cos(psi);
    x   = cumtrapz(tout, dx);
    y   = cumtrapz(tout, dy);

    subplot(1,3,i);
    plot(x_ref, y_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference'); hold on;
    plot(x, y, 'b-', 'LineWidth', 1, 'DisplayName', 'PD');
    xlabel('X (m)'); ylabel('Y (m)');
    title(titles{i}); legend; grid on; axis equal;
end
sgtitle('Circular Trajectory - PD Controller');

%% PDFC 결과 플롯 (Fig.17-19)
figure('Name', 'Fig.17-19 PDFC', 'Position', [100, 550, 1100, 400]);
for i = 1:3
    res    = results_curv_PDFC(i);
    tout   = res.tout;
    dtheta = res.dq(:,1);
    beta   = res.q(:,4);

    e_actual = R .* dtheta.^2 .* (Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) ...
               ./ (mp*g*l*sin(beta) + 1e-10);
    e_actual = max(min(e_actual, 10*e_target), 0.1*e_target);
    psi = cumtrapz(tout, -R * dtheta ./ e_actual);
    dx  = R * dtheta .* sin(psi);
    dy  = -R * dtheta .* cos(psi);
    x   = cumtrapz(tout, dx);
    y   = cumtrapz(tout, dy);

    subplot(1,3,i);
    plot(x_ref, y_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference'); hold on;
    plot(x, y, 'r-', 'LineWidth', 1, 'DisplayName', 'PDFC');
    xlabel('X (m)'); ylabel('Y (m)');
    title(titles{i}); legend; grid on; axis equal;
end
sgtitle('Circular Trajectory - PD-Fuzzy Controller');

%% 오차 비교 출력
% phi 추종 오차: 올바른 참조값 phi_d = (v_d/(R*Omega))*(1-cos(Omega*t)) 기준
fprintf('=== 궤적 추종 오차 비교 (phi 기준) ===\n');
fprintf('%-10s %-15s %-15s\n', 'dt', 'PD phi 오차', 'PDFC phi 오차');
for i = 1:3
    phi_PD   = results_curv_PD(i).q(:,3);
    phi_PDFC = results_curv_PDFC(i).q(:,3);
    t_PD     = results_curv_PD(i).tout;
    t_PDFC   = results_curv_PDFC(i).tout;

    phi_d_PD   = (v_d / (R * Omega)) * (1 - cos(Omega * t_PD));
    phi_d_PDFC = (v_d / (R * Omega)) * (1 - cos(Omega * t_PDFC));

    err_PD   = max(abs(phi_PD   - phi_d_PD));
    err_PDFC = max(abs(phi_PDFC - phi_d_PDFC));

    fprintf('dt=%.3f:  PD=%.4f rad,  PDFC=%.4f rad\n', ...
        results_curv_PD(i).dt, err_PD, err_PDFC);
end
end
```

> **오차 기준**: `phi_d = Omega*t`는 구르기 조건을 무시한 잘못된 참조값이다. 올바른 참조값은 구르기 조건 $v_y = R\dot{\phi}$과 원운동 기하학 $v_y = v_d\sin(\Omega t)$로부터 유도한 $\phi_d(t) = \frac{v_d}{R\Omega}(1-\cos(\Omega t))$이다.

---

## 8. 전체 실행 방법

### 8.1 `run_all.m`

```matlab
% run_all.m
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'core'));
addpath(fullfile(root, 'models'));
addpath(fullfile(root, 'simulations'));
addpath(fullfile(root, 'plots'));
addpath(fullfile(root, 'results'));
cd(root);

fprintf('=== 전체 시뮬레이션 시작 ===\n\n');

load_system('SRR_PD');
load_system('SRR_PDFC');

%% 1. 직선 궤적 시뮬레이션
fprintf('--- [1/2] 직선 궤적 ---\n');
fprintf('SRR_PD, SRR_PDFC의 Traj_gen이 traj_linear로 되어 있는지 확인하세요.\n\n');
run_linear;
run_linear_PDFC;
plot_results;
plot_linear_comparison;

%% 2. 곡선 궤적 전환 안내
fprintf('\n========================================\n');
fprintf('[사용자 액션 필요]\n');
fprintf('1. models/SRR_PD.slx 열기\n');
fprintf('2. Traj_gen 블록 더블클릭\n');
fprintf('3. traj_linear → traj_circular 로 변경\n');
fprintf('4. 에디터 닫기 → Ctrl+S 저장\n');
fprintf('5. SRR_PDFC.slx 도 동일하게\n');
fprintf('6. Command Window 클릭 후 Enter\n');
fprintf('========================================\n');
pause;

bdclose('SRR_PD'); bdclose('SRR_PDFC');
load_system('SRR_PD'); load_system('SRR_PDFC');

%% 3. 곡선 궤적 시뮬레이션
fprintf('\n--- [2/2] 곡선 궤적 ---\n');
run_curv_PD;
run_curv_PDFC;
plot_curvilinear;
plot_curvilinear_comparison;

%% 4. 복원 안내
fprintf('\n========================================\n');
fprintf('[복원] Traj_gen을 traj_linear로 되돌리고 Enter\n');
fprintf('========================================\n');
pause;

fprintf('\n=== 전체 시뮬레이션 완료 ===\n');
```

### 8.2 단계별 실행 순서

**Step 1**: MATLAB Current Folder를 `Kayacan_SRR_matsim` 루트로 설정

**Step 2**: `run_all` 실행

**Step 3**: 직선 궤적 시뮬레이션 자동 실행. 총 5개 figure 생성 (Fig.8 Kp 변화, Fig.9 Kv 변화, Kp=1 Kv=1 상세, Fig.12 PD vs PDFC, 속도 오차 비교). 모든 figure는 닫히지 않고 유지된다.

**Step 4**: Command Window에 안내 메시지가 뜨면 두 Simulink 모델의 `Traj_gen` 블록 코드를 `traj_circular`로 변경 후 저장 → **Enter**

**Step 5**: 곡선 궤적 시뮬레이션 자동 실행. 추가로 3개 figure 생성 (Fig.14-16 PD, Fig.17-19 PDFC). 이전 figure와 함께 총 8개 figure가 모두 열려 있다.

**Step 6**: 다시 안내 메시지가 뜨면 `traj_linear`로 복원 후 **Enter**

> **figure 관리**: `run_all.m` 실행 중 생성된 모든 figure는 자동으로 닫히지 않는다. 확인 후 직접 닫거나, 다음 실행 전 Command Window에서 `close all`을 입력한다.

### 8.3 시뮬레이션 스크립트 단독 실행

각 시뮬레이션 스크립트를 개별적으로 실행할 수도 있다. 단독 실행 시에도 `run_all.m`과 동일한 결과가 생성된다.

**전제 조건**: MATLAB Current Folder가 `Kayacan_SRR_matsim` 루트여야 한다.

```matlab
% 직선 궤적 (SRR_PD, Kp/Kv 변화)
run_linear()

% 직선 궤적 (SRR_PDFC)
run_linear_PDFC()

% 곡선 궤적 (SRR_PD) — Traj_gen이 traj_circular인 상태에서 실행
run_curv_PD()

% 곡선 궤적 (SRR_PDFC) — Traj_gen이 traj_circular인 상태에서 실행
run_curv_PDFC()
```

결과를 그리려면:

```matlab
plot_results()
plot_linear_comparison()
plot_curvilinear()
plot_curvilinear_comparison()
```

---

## 9. 결과 해석

### 직선 궤적 (논문 Fig.8, 9)

| 현상 | 원인 | 해석 |
|---|---|---|
| $K_p$가 클수록 오버슈트 증가 | 비례 이득이 크면 응답이 빠르지만 진동 증가 | Fig.8 패턴 |
| $K_v$가 클수록 오버슈트 감소 | 미분 이득이 크면 진동 억제 | Fig.9 패턴 |
| PDFC가 PD보다 오버슈트 작음 | 퍼지가 초반 큰 오차 구간에서 이득 조절 | Fig.12 패턴 |

- 정착 시간: PD ≈ 1.17 s, PDFC ≈ 0.90 s (Kp=1, Kv=1 기준)

### 곡선 궤적 (논문 Fig.14-19)

- **PD**: 반지름 1 m 원을 Reference와 거의 완벽하게 추종. 샘플링 주기 0.001 s → 0.5 s → 1.0 s 순서로 궤적 오차가 점차 증가하는 양상이 나타남.
- **PDFC**: 현재 퍼지 이득 스케일 설정상 PD보다 오차가 다소 크게 나타남. 퍼지 룰 베이스 튜닝으로 개선 가능.

### X-Y 궤적 계산 방법

`phi`(y축 기준 구 회전각)는 heading angle이 아니므로 XY 적분에 직접 사용할 수 없다. 논문 식 (33) `Ω = -Rθ̇/e`를 이용해 heading angle `psi`를 적분 복원하고, 식 (39)로 실제 beta에서 `e_actual`을 매 시점 계산해 사용한다:

$$\dot{\psi} = -\frac{R\dot{\theta}}{e_\text{actual}}, \quad \dot{x} = R\dot{\theta}\sin\psi, \quad \dot{y} = -R\dot{\theta}\cos\psi$$

```matlab
% 식 (39): 실제 beta로 매 시점 곡률 반경 계산
e_actual = R .* dtheta.^2 .* (Is - mp*R*l*cos(beta) + R^2*(Ms+mp)) ...
           ./ (mp*g*l*sin(beta) + 1e-10);
e_actual = max(min(e_actual, 10*e_target), 0.1*e_target);

% 식 (33): heading angle 적분
psi = cumtrapz(tout, -R * dtheta ./ e_actual);
x   = cumtrapz(tout, R * dtheta .* sin(psi));
y   = cumtrapz(tout, -R * dtheta .* cos(psi));
```

> **주의**: `e_actual` 대신 `e_target`(고정값)을 쓰면 beta 오차와 무관하게 heading이 reference와 동일해져 XY 궤적이 항상 완벽한 원으로 보이는 버그가 생긴다.

---

## 10. 주의사항 및 트러블슈팅

### Traj_gen 전환 시 반드시 모델 저장

`Traj_gen` 블록 코드를 변경한 후 **반드시 Ctrl+S로 모델을 저장**해야 한다. 저장하지 않으면 `run_all.m`에서 `bdclose` 후 `load_system`으로 다시 불러올 때 변경 전 코드가 로드된다.

### To Workspace 저장 형식 확인

To Workspace 블록의 "2차원 신호를 다음으로 저장" 옵션이 **2차원 배열(첫 번째 차원을 따라 결합)**이어야 한다. 이 설정이 잘못되어 있으면:

```
오류: 좌표를 동일한 크기의 벡터 또는 행렬로 지정하십시오
```

같은 에러가 플롯 스크립트에서 발생한다.

### reshape 방향

`simOut.q_out`이 열벡터로 반환되는 경우 reshape 방향이 중요하다:

```matlab
% 올바른 방법: 4×N 행렬로 reshape 후 전치
q = reshape(simOut.q_out, 4, N)';   % → N×4

% 잘못된 방법: N×4로 직접 reshape (채널 순서가 뒤섞임)
q = reshape(simOut.q_out, N, 4);    % ← 이렇게 하면 안 됨
```

### Kp, Kv 경고 (빨간 삼각형)

Simulink 모델을 처음 열 때 workspace에 `Kp`, `Kv`가 없으면 Gain 블록에 빨간 삼각형 경고가 뜬다.

**해결 방법 1**: `run_all.m`을 통해 실행 (자동 처리됨)

**해결 방법 2**: Command Window에서 먼저 `Kp=1; Kv=1;` 입력

**해결 방법 3**: 모델 InitFcn 콜백에 `Kp=1; Kv=1;` 설정 (영구 해결)

### MATLAB 경로 문제

`run_all.m`을 루트 폴더에서 실행하지 않으면 `core/`, `models/` 등 하위 폴더의 함수를 찾지 못해 에러가 난다. 반드시 `Kayacan_SRR_matsim` 폴더를 MATLAB Current Folder로 설정하거나 `cd`로 이동한 후 실행한다.

---

## 참고: 논문 Figure와 코드 대응표

| 논문 Figure | 내용 | 대응 파일 |
|---|---|---|
| Fig.5 | PD 제어기 블록 다이어그램 | `models/SRR_PD.slx` |
| Fig.6 | PDFC 블록 다이어그램 | `models/SRR_PDFC.slx` |
| Fig.8 | Kp 변화, 직선 궤적 속도 응답 | `run_linear.m` + `plot_results.m` |
| Fig.9 | Kv 변화, 직선 궤적 속도 응답 | `run_linear.m` + `plot_results.m` |
| Fig.12 | PD vs PDFC 직선 궤적 비교 | `run_linear_PDFC.m` + `plot_linear_comparison.m` |
| Fig.14-16 | PD 곡선 궤적, 샘플링 주기 비교 | `run_curv_PD.m` + `plot_curvilinear.m` |
| Fig.17-19 | PDFC 곡선 궤적, 샘플링 주기 비교 | `run_curv_PDFC.m` + `plot_curvilinear_comparison.m` |
