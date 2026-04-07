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

titles = {'dt = 0.001 s', 'dt = 0.1 s', 'dt = 0.15 s'};

for i = 1:3
    res  = results_curv_PD(i);
    tout = res.tout;

    dtheta = res.dq(:,1);

    % heading angle psi: psi_dot = -R*dtheta/e (식 33: Omega = -R*dtheta/e)
    % phi는 y축 기준 구 회전각(lateral rolling angle)으로 heading angle이 아님
    psi = cumtrapz(tout, -R * dtheta / e_target);

    % 글로벌 XY: 구르기 속도 v = R*|dtheta|, 방향은 heading psi
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