function [ref_ddq, ref_dq, ref_q] = traj_circular(t)
% 원형 궤적 생성
% 논문 Fig.14-19 재현
%
% 원형 궤적 조건:
%   - 구동 속도: dtheta_d = 2.5 rad/s (v=0.5 m/s)
%   - 곡률 반경: e = 1.0 m (반지름 1m 원)
%   - beta: 식 (39)로부터 역산
%
% phi 목표값 유도 (구르기 조건으로부터):
%   v_y = R * phi_dot  (구르기 조건)
%   v_y = v_d * sin(Omega*t)  (원운동 기하학)
%   => phi_dot_d = (v_d/R) * sin(Omega*t)
%   => phi_d     = (v_d/(R*Omega)) * (1 - cos(Omega*t))
%   => phi_ddot_d = (v_d/R) * Omega * cos(Omega*t)
%   Omega < 0이므로 v_d/(R*Omega) < 0 (phi는 음수 방향으로 기울어짐)

%% 파라미터
R   = 0.2;
Ms  = 3;
mp  = 2;
Is  = (2/3) * Ms * R^2;
l   = 0.075;
g   = 9.81;

v_d      = 0.5;
dtheta_d = v_d / R;   % = 2.5 rad/s
e_target = 1.0;       % 목표 곡률 반경 [m]

%% 식 (39)에서 beta 역산
% e = R*dtheta^2 * [Is - mp*R*l*cos(beta) + R^2*(Ms+mp)] / (mp*g*l*sin(beta))
% beta를 수치적으로 풀기
beta = fzero(@(b) ...
    R * dtheta_d^2 * (Is - mp*R*l*cos(b) + R^2*(Ms+mp)) ...
    / (mp*g*l*sin(b)+1e-10) - e_target, ...
    0.3);

%% Omega (수직축 각속도)
Omega = -R * dtheta_d / e_target;

%% 목표 궤적
% x-subsystem: 구동 (theta)
% y-subsystem: 조향 (phi, beta)
phi_d      = (v_d / (R * Omega)) * (1 - cos(Omega * t));
phi_dot_d  = (v_d / R) * sin(Omega * t);
phi_ddot_d = (v_d / R) * Omega * cos(Omega * t);

ref_q   = [dtheta_d * t; 0; phi_d; beta];
ref_dq  = [dtheta_d; 0; phi_dot_d; 0];
ref_ddq = [0; 0; phi_ddot_d; 0];
end