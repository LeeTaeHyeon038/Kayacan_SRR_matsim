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