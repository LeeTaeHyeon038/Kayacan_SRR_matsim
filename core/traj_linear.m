function [ref_ddq, ref_dq, ref_q] = traj_linear(t)
% 직선 궤적 생성
v_d      = 0.5;
R        = 0.2;
dtheta_d = v_d / R;

ref_q   = [dtheta_d * t; 0; 0; 0];
ref_dq  = [dtheta_d; 0; 0; 0];
ref_ddq = [0; 0; 0; 0];
end