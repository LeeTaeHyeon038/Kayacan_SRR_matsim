% main_sim.m
% 구형 로봇 시뮬레이션 메인 스크립트
% Kayacan et al. (2012) 논문 Section 5 재현
% 직선 궤적 추종 시뮬레이션

clear; clc; close all;

%% 1. 파라미터 로드 및 구조체 구성
params;   % params.m 실행 -> Ms, mp, R, l, g, Is, Ip, Kp, Kv, u_max 로드

p.Ms    = Ms;
p.mp    = mp;
p.R     = R;
p.l     = l;
p.g     = g;
p.Is    = Is;
p.Ip    = Ip;
p.Kp    = Kp;
p.Kv    = Kv;
p.u_max = u_max;

%% 2. 초기 상태 설정
% state = [theta; alpha; phi; beta; dtheta; dalpha; dphi; dbeta]
% 정적 평형 상태에서 시작 (모든 각도, 각속도 = 0)
state0 = zeros(8, 1);

%% 3. 목표 궤적 정의 (직선 궤적: 속도 step 입력)
% 논문 Fig.8, 9 재현: 속도를 0에서 0.5 m/s로 step
% 구 선속도 v = R * dtheta 이므로 dtheta_d = v_d / R
v_d     = 0.5;            % 목표 선속도 [m/s]
dtheta_d = v_d / R;       % 목표 각속도 [rad/s]

% 목표 궤적 함수: q_d = [theta_d; alpha_d; phi_d; beta_d]
% 직선 주행: theta만 변화, 나머지는 0 유지
% alpha_d = 0 (진자는 수직 유지)
traj_func = @(t) [
    dtheta_d * t;   % theta_d  : 등속도로 증가
    0;              % alpha_d  : 진자 수직
    0;              % phi_d    : y방향 정지
    0;              % beta_d   : 진자 수직
    dtheta_d;       % dtheta_d
    0;              % dalpha_d
    0;              % dphi_d
    0;              % dbeta_d
    0;              % ddtheta_d (등속이므로 0)
    0;              % ddalpha_d
    0;              % ddphi_d
    0               % ddbeta_d
];

%% 4. ODE 적분
t_span = [0, 20];                        % 시뮬레이션 시간 [s]
opts   = odeset('RelTol', 1e-6, ...
                'AbsTol', 1e-8, ...
                'MaxStep', 0.001);       % 논문 샘플링 주기 0.001s

fprintf('시뮬레이션 실행 중...\n');
[t, state] = ode45(@(t, s) dynamics(t, s, p, traj_func), ...
                   t_span, state0, opts);
fprintf('완료.\n');

%% 5. 결과 추출
theta  = state(:,1);
alpha  = state(:,2);
phi    = state(:,3);
beta   = state(:,4);
dtheta = state(:,5);
dalpha = state(:,6);
dphi   = state(:,7);
dbeta  = state(:,8);

% 구 선속도
v_sphere = R * abs(dtheta);

% 목표 속도
v_ref = v_d * ones(size(t));

%% 6. 결과 플롯
figure('Name', 'Spherical Robot Simulation', 'NumberTitle', 'off');

% 속도 추종
subplot(2,2,1);
plot(t, v_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference'); hold on;
plot(t, v_sphere, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Tracking (Linear Trajectory)');
legend; grid on;

% theta 추종
subplot(2,2,2);
traj_vals = arrayfun(@(ti) traj_func(ti), t, 'UniformOutput', false);
theta_d = cellfun(@(x) x(1), traj_vals);
plot(t, theta_d, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference'); hold on;
plot(t, theta,   'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual');
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Theta Tracking');
legend; grid on;

% 진자 각도 alpha
subplot(2,2,3);
plot(t, alpha * (180/pi), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\alpha (deg)');
title('Pendulum Angle \alpha');
grid on;

% 속도 오차
subplot(2,2,4);
e_v = v_ref - v_sphere;
plot(t, e_v, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Error');
grid on;

fprintf('\n=== 결과 요약 ===\n');
fprintf('최대 속도 오차: %.4f m/s\n', max(abs(e_v)));
fprintf('정착 시간 근사 (|e| < 0.01): ');
idx = find(abs(e_v) < 0.01, 1, 'first');
if ~isempty(idx)
    fprintf('%.2f s\n', t(idx));
else
    fprintf('수렴 안됨\n');
end
