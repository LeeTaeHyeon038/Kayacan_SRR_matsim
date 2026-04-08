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