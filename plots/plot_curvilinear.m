% plot_curvilinear.m
% 논문 Fig.14-16 재현: 원형 궤적 추종 결과

clear; clc; close all;
addpath('results');
addpath('core');

R        = 0.2;
v_d      = 0.5;
e_target = 1.0;
Omega    = -v_d / e_target;

%% 목표 원형 궤적
t_ref   = linspace(0, 20, 2000);
phi_ref = Omega * t_ref;
x_ref   = cumtrapz(t_ref, v_d * sin(phi_ref));
y_ref   = cumtrapz(t_ref, -v_d * cos(phi_ref));

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
    phi    = res.q(:,3);

    % 구르기 조건으로 x, y 적분
    dx = R * dtheta .* sin(phi);
    dy = -R * dtheta .* cos(phi);

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