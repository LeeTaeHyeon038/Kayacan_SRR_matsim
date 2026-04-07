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
%% ============================================================
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