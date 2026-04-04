% run_all.m
% 전체 시뮬레이션 마스터 스크립트
% 이것만 실행하면 모든 결과가 생성됨

clear; clc; close all;

%% 경로 설정
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'core'));
addpath(fullfile(root, 'models'));
addpath(fullfile(root, 'simulations'));
addpath(fullfile(root, 'plots'));
addpath(fullfile(root, 'results'));
cd(root);

fprintf('=== 전체 시뮬레이션 시작 ===\n\n');

%% 1. 직선 궤적 시뮬레이션
fprintf('--- [1/2] 직선 궤적 시뮬레이션 시작 ---\n');
fprintf('SRR_PD, SRR_PDFC의 Traj_gen이 traj_linear로 되어 있어야 합니다.\n\n');

load_system('SRR_PD');
load_system('SRR_PDFC');

run_linear;
run_linear_PDFC;
plot_results;
plot_linear_comparison;

fprintf('\n직선 궤적 시뮬레이션 완료.\n');

%% 2. 곡선 궤적 전환 안내
fprintf('\n========================================\n');
fprintf('[사용자 액션 필요]\n');
fprintf('1. models/SRR_PD.slx 열기\n');
fprintf('2. Traj_gen 블록 더블클릭\n');
fprintf('3. 코드에서 traj_linear -> traj_circular 로 변경\n');
fprintf('4. 에디터 닫기\n');
fprintf('5. Ctrl+S 로 모델 저장\n');
fprintf('6. SRR_PDFC.slx 도 동일하게 반복\n');
fprintf('7. 완료되면 여기 Command Window 클릭 후 Enter\n');
fprintf('========================================\n');
pause;

%% 모델 닫고 다시 로드 (변경사항 반영)
bdclose('SRR_PD');
bdclose('SRR_PDFC');
load_system('SRR_PD');
load_system('SRR_PDFC');

%% 3. 곡선 궤적 시뮬레이션
fprintf('\n--- [2/2] 곡선 궤적 시뮬레이션 시작 ---\n');

run_curv_PD;
run_curv_PDFC;
plot_curvilinear;
plot_curvilinear_comparison;

fprintf('\n곡선 궤적 시뮬레이션 완료.\n');

%% 4. 복원 안내
fprintf('\n========================================\n');
fprintf('[사용자 액션 필요 - 복원]\n');
fprintf('1. models/SRR_PD.slx 열기\n');
fprintf('2. Traj_gen 블록 더블클릭\n');
fprintf('3. 코드에서 traj_circular -> traj_linear 로 변경\n');
fprintf('4. 에디터 닫기\n');
fprintf('5. Ctrl+S 로 모델 저장\n');
fprintf('6. SRR_PDFC.slx 도 동일하게 반복\n');
fprintf('7. 완료되면 여기 Command Window 클릭 후 Enter\n');
fprintf('========================================\n');
pause;

fprintf('\n=== 전체 시뮬레이션 완료 ===\n');