function [ref_ddq, ref_dq, ref_q] = traj_circular(t)
% 원형 궤적 생성
% 논문 Fig.14-19 재현
%
% 원형 궤적 조건:
%   - 구동 속도: dtheta_d = 2.5 rad/s (v=0.5 m/s)
%   - 곡률 반경: e = 1.0 m (반지름 1m 원)
%   - beta: 식 (39)로부터 역산

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
phi_d = Omega * t;   % phi는 Omega로 적분

ref_q   = [dtheta_d * t; 0; phi_d; beta];
ref_dq  = [dtheta_d; 0; Omega; 0];
ref_ddq = [0; 0; 0; 0];
end