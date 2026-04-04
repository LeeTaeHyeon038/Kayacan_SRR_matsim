% run_linear.m
% 논문 Fig.8, 9, 12 재현
% 여러 Kp, Kv 조건으로 시뮬레이션 반복 실행

clear; clc;
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
    assignin('base', 'Kp', Kp);   % ← 추가
    assignin('base', 'Kv', Kv);   % ← 추가
    simOut = sim(mdl, 'StopTime', '20');

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
    assignin('base', 'Kp', Kp);   % ← 추가
    assignin('base', 'Kv', Kv);   % ← 추가
    simOut = sim(mdl, 'StopTime', '20');

    results_Kv(i).Kp   = Kp;
    results_Kv(i).Kv   = Kv;
    results_Kv(i).tout = simOut.tout;
    results_Kv(i).q    = simOut.q_out;
    results_Kv(i).dq   = simOut.dq_out;
end

%% 결과 저장
save('results_linear.mat', 'results_Kp', 'results_Kv');
fprintf('\n결과 저장 완료: results_linear.mat\n');