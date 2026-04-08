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