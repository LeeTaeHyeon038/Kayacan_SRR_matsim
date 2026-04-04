function u = control_input(q, dq, q_d, dq_d, ddq_d, params)
% control_input.m
% 피드백 선형화 제어 입력 u(t) 계산
% 논문 식 (41) 참조:
%   u(t) = V(q, dq) + Kv*de + Kp*e + M(q)*ddq_d
%
% 입력:
%   q      : 현재 일반좌표 [theta; alpha; phi; beta] (4x1)
%   dq     : 현재 일반좌표 미분 (4x1)
%   q_d    : 목표 궤적 (4x1)
%   dq_d   : 목표 속도 (4x1)
%   ddq_d  : 목표 가속도 (4x1)
%   params : 시스템 파라미터 구조체
%
% 출력:
%   u      : 제어 입력 토크 [tau_x; tau_x; tau_y; tau_y] (4x1)

%% 파라미터 추출
Kp    = params.Kp;
Kv    = params.Kv;
u_max = params.u_max;

%% 오차 계산
e  = q_d  - q;    % 위치 오차
de = dq_d - dq;   % 속도 오차

%% M, V 계산
M = calc_M(q, params);
V = calc_V(q, dq, params);

%% 피드백 선형화 제어 입력 (논문 식 41)
%   u = V + Kv*de + Kp*e + M*ddq_d
u = V + Kv*de + Kp*e + M*ddq_d;

%% 토크 포화 적용 (saturation, 논문 Section 5)
u = max(-u_max, min(u_max, u));

end
