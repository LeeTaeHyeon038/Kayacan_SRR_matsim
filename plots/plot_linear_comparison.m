% plot_linear_comparison.m
% 논문 Fig.12 재현: PD vs PD-Fuzzy 비교

clear; clc; close all;
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