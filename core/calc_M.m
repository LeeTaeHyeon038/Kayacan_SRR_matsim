function M = calc_M(q, params)
% calc_M.m
% 관성 행렬 M(q) 계산
% 논문 식 (31), (32) 참조
%
% 입력:
%   q      : 일반좌표 벡터 [theta; alpha; phi; beta] (4x1)
%   params : 시스템 파라미터 구조체
%
% 출력:
%   M      : 4x4 관성 행렬
%
% 일반좌표 정의:
%   q1 = theta  (x축 기준 구 회전각)
%   q2 = alpha  (x축 기준 진자 회전각)
%   q3 = phi    (y축 기준 구 회전각)
%   q4 = beta   (y축 기준 진자 회전각)

%% 파라미터 추출
Ms = params.Ms;
mp = params.mp;
R  = params.R;
l  = params.l;
Is = params.Is;
Ip = params.Ip;

%% 일반좌표 추출
q1 = q(1);   % theta
q2 = q(2);   % alpha
q3 = q(3);   % phi
q4 = q(4);   % beta

%% M 행렬 성분 계산
% x-subsystem (1,2행/열): theta, alpha
M11 = Ms*R^2 + mp*R^2 + mp*l^2 + Is + Ip + 2*mp*R*l*cos(q2 - q1);
M12 = -mp*l^2 - Ip - mp*R*l*cos(q2 - q1);
M21 = M12;
M22 = mp*l^2 + Ip;

% y-subsystem (3,4행/열): phi, beta
M33 = Ms*R^2 + mp*R^2 + mp*l^2 + Is + Ip + 2*mp*R*l*cos(q4 - q3);
M34 = -mp*l^2 - Ip - mp*R*l*cos(q4 - q3);
M43 = M34;
M44 = mp*l^2 + Ip;

%% 행렬 조립
% 블록 대각 구조: x-subsystem과 y-subsystem이 완전히 분리됨
M = [M11, M12,   0,   0;
     M21, M22,   0,   0;
       0,   0, M33, M34;
       0,   0, M43, M44];
end
