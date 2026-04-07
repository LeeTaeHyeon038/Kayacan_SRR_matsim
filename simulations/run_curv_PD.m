% run_curv_PD.m
% 논문 Fig.14, 15, 16 재현
% 원형 궤적 + PD 제어, 샘플링 주기 3가지

function run_curv_PD()
clear; clc;
addpath('core');
addpath('models');
mdl = 'SRR_PD';
load_system(mdl);
v_d = 0.5; e_target = 1.0;
Omega = -v_d / e_target;
T_sim = 2*pi / abs(Omega);   % 원 한 바퀴 주기 ≈ 12.57 s
step_list = [0.001, 0.1, 0.15];
results_curv_PD = struct();
fprintf('=== 곡선 궤적 PD 시뮬레이션 ===\n');

for i = 1:length(step_list)
    dt = step_list(i);
    fprintf('샘플링 주기 %.3f s 실행 중...\n', dt);
    
    % --- 수정된 부분 (안전한 파라미터 전달) ---
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setVariable('Kp', 1.0);
    simIn = simIn.setVariable('Kv', 1.0);
    simIn = simIn.setModelParameter('FixedStep', num2str(dt));
    simIn = simIn.setModelParameter('StopTime', num2str(T_sim));
    simOut = sim(simIn);
    % -----------------------------------------
    
    N  = length(simOut.tout);
    q  = reshape(simOut.q_out,  4, N)';
    dq = reshape(simOut.dq_out, 4, N)';
    results_curv_PD(i).dt   = dt;
    results_curv_PD(i).tout = simOut.tout;
    results_curv_PD(i).q    = q;
    results_curv_PD(i).dq   = dq;
end

save('results/results_curv_PD.mat', 'results_curv_PD');
fprintf('결과 저장 완료: results_curv_PD.mat\n');
end