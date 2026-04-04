% plot_curvilinear_comparison.m
% 논문 Fig.14-19 재현: PD vs PDFC 원형 궤적 비교

clear; clc; close all;
addpath('results');

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
load('results/results_curv_PDFC.mat');

titles = {'dt = 0.001 s', 'dt = 0.1 s', 'dt = 0.15 s'};

%% PD 결과 플롯 (Fig.14-16)
figure('Name', 'Fig.14-16 PD', 'Position', [100, 100, 1100, 400]);
for i = 1:3
    res    = results_curv_PD(i);
    tout   = res.tout;
    dtheta = res.dq(:,1);
    phi    = res.q(:,3);

    dx = R * dtheta .* sin(phi);
    dy = -R * dtheta .* cos(phi);
    x  = cumtrapz(tout, dx);
    y  = cumtrapz(tout, dy);

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
    phi    = res.q(:,3);

    dx = R * dtheta .* sin(phi);
    dy = -R * dtheta .* cos(phi);
    x  = cumtrapz(tout, dx);
    y  = cumtrapz(tout, dy);

    subplot(1,3,i);
    plot(x_ref, y_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference'); hold on;
    plot(x, y, 'r-', 'LineWidth', 1, 'DisplayName', 'PDFC');
    xlabel('X (m)'); ylabel('Y (m)');
    title(titles{i}); legend; grid on; axis equal;
end
sgtitle('Circular Trajectory - PD-Fuzzy Controller');

%% 오차 비교 출력
fprintf('=== 궤적 추종 오차 비교 ===\n');
fprintf('%-10s %-15s %-15s\n', 'dt', 'PD phi 오차', 'PDFC phi 오차');
for i = 1:3
    phi_PD   = results_curv_PD(i).q(:,3);
    phi_PDFC = results_curv_PDFC(i).q(:,3);
    t_PD     = results_curv_PD(i).tout;
    t_PDFC   = results_curv_PDFC(i).tout;

    err_PD   = max(abs(phi_PD   - Omega * t_PD));
    err_PDFC = max(abs(phi_PDFC - Omega * t_PDFC));

    fprintf('dt=%.3f:  PD=%.4f rad,  PDFC=%.4f rad\n', ...
        results_curv_PD(i).dt, err_PD, err_PDFC);
end