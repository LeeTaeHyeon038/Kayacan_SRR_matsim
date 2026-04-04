% run_curv_PDFC.m
% 논문 Fig.17, 18, 19 재현
% 원형 궤적 + PD-Fuzzy 제어, 샘플링 주기 3가지

clear; clc;
addpath('core');
addpath('models');

mdl = 'SRR_PDFC';
load_system(mdl);

step_list = [0.001, 0.1, 0.15];
results_curv_PDFC = struct();

fprintf('=== 곡선 궤적 PDFC 시뮬레이션 ===\n');
for i = 1:length(step_list)
    dt = step_list(i);
    fprintf('샘플링 주기 %.3f s 실행 중...\n', dt);

    set_param(mdl, 'FixedStep', num2str(dt));

    simOut = sim(mdl, 'StopTime', '20');

    N  = length(simOut.tout);
    q  = reshape(simOut.q_out,  4, N)';
    dq = reshape(simOut.dq_out, 4, N)';

    results_curv_PDFC(i).dt   = dt;
    results_curv_PDFC(i).tout = simOut.tout;
    results_curv_PDFC(i).q    = q;
    results_curv_PDFC(i).dq   = dq;
end

save('results/results_curv_PDFC.mat', 'results_curv_PDFC');
fprintf('결과 저장 완료: results_curv_PDFC.mat\n');