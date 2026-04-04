function dstate = dynamics(t, state, params, traj_func)
% dynamics.m
% ode45에 넘길 상태방정식 정의
% M(q)*ddq + V(q,dq) = u 에서 ddq = M^{-1}*(u - V)
%
% 상태벡터 정의:
%   state = [q; dq] = [theta; alpha; phi; beta; dtheta; dalpha; dphi; dbeta] (8x1)
%
% 입력:
%   t         : 현재 시간
%   state     : 현재 상태벡터 (8x1)
%   params    : 시스템 파라미터 구조체
%   traj_func : 목표 궤적 함수 핸들 @(t) -> [q_d; dq_d; ddq_d] (12x1)
%
% 출력:
%   dstate    : 상태벡터 미분 [dq; ddq] (8x1)

%% 상태벡터 분리
q  = state(1:4);   % 위치: [theta; alpha; phi; beta]
dq = state(5:8);   % 속도: [dtheta; dalpha; dphi; dbeta]

%% 목표 궤적 계산
traj   = traj_func(t);
q_d    = traj(1:4);    % 목표 위치
dq_d   = traj(5:8);    % 목표 속도
ddq_d  = traj(9:12);   % 목표 가속도

%% M, V 계산
M = calc_M(q, params);
V = calc_V(q, dq, params);

%% 제어 입력 계산
u = control_input(q, dq, q_d, dq_d, ddq_d, params);

%% 운동방정식 풀기: ddq = M^{-1} * (u - V)
ddq = M \ (u - V);   % M\b 가 inv(M)*b 보다 수치적으로 안정적

%% 상태벡터 미분 조립
dstate = [dq; ddq];

end
