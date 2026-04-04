function V = calc_V(q, dq, params)
% calc_V.m
% 비선형 항 벡터 V(q, dq) 계산
% 논문 식 (31), (32) 참조
%
% 입력:
%   q      : 일반좌표 벡터 [theta; alpha; phi; beta] (4x1)
%   dq     : 일반좌표 미분 벡터 [dtheta; dalpha; dphi; dbeta] (4x1)
%   params : 시스템 파라미터 구조체
%
% 출력:
%   V      : 4x1 비선형 항 벡터 (원심력, 코리올리 힘, 중력 포함)
%
% V 벡터 물리적 의미:
%   dq^2 항  -> 원심력 효과
%   dq1*dq2  -> 코리올리 힘 (계수 2가 특징)
%   sin 항   -> 중력에 의한 복원력

%% 파라미터 추출
mp = params.mp;
R  = params.R;
l  = params.l;
g  = params.g;

%% 일반좌표 및 미분 추출
q1 = q(1);    % theta
q2 = q(2);    % alpha
q3 = q(3);    % phi
q4 = q(4);    % beta

dq1 = dq(1);  % dtheta/dt
dq2 = dq(2);  % dalpha/dt
dq3 = dq(3);  % dphi/dt
dq4 = dq(4);  % dbeta/dt

%% V 벡터 성분 계산
% x-subsystem: theta, alpha
% sin(q2-q1) = sin(alpha - theta)
s21 = sin(q2 - q1);

% V11: 구(theta) 방정식의 비선형 항
%   dq1^2 항 : theta 회전에 의한 원심력
%   dq2^2 항 : alpha 회전에 의한 원심력
%   dq1*dq2  : 코리올리 힘 (계수 2)
%   중력 항  : 복원력
V11 = mp*R*l*s21*dq1^2 ...
    + mp*R*l*s21*dq2^2 ...
    - 2*mp*R*l*s21*dq1*dq2 ...
    - mp*g*l*s21;

% V21: 진자(alpha) 방정식의 비선형 항 (중력 복원력만)
V21 = mp*g*l*s21;

% y-subsystem: phi, beta
% sin(q4-q3) = sin(beta - phi)
s43 = sin(q4 - q3);

% V31: 구(phi) 방정식의 비선형 항
V31 = mp*R*l*s43*dq3^2 ...
    + mp*R*l*s43*dq4^2 ...
    - 2*mp*R*l*s43*dq3*dq4 ...
    - mp*g*l*s43;

% V41: 진자(beta) 방정식의 비선형 항 (중력 복원력만)
V41 = mp*g*l*s43;

%% 벡터 조립
V = [V11; V21; V31; V41];
end
