% params.m
% Kayacan et al. (2012) 논문 시스템 파라미터 정의
% 논문 Section 5 수치값 사용

%% 시스템 물리 파라미터
Ms = 3;          % 구의 질량 [kg]
mp = 2;          % 진자의 질량 [kg]
R  = 0.2;        % 구의 반지름 [m]
l  = 0.075;      % 진자의 길이 [m]
g  = 9.81;       % 중력가속도 [m/s^2]

%% 관성 모멘트
% 구: hollow spherical shell -> (2/3)*Ms*R^2
Is = (2/3) * Ms * R^2;

% 진자: slender rod, 회전축 = 구 중심
% 평행축 정리 적용: I_cm + mp*(l/2)^2 = (1/12)*mp*l^2 + mp*(l/2)^2 = (1/3)*mp*l^2
Ip = (1/3) * mp * l^2;

%% 제어 파라미터
Kp = 1;          % proportional gain
Kv = 1;          % velocity gain
u_max = 2.5;     % 최대 입력 토크 [Nm] (saturation)

%% 확인 출력
fprintf('=== 시스템 파라미터 ===\n');
fprintf('Ms = %.2f kg,  mp = %.2f kg\n', Ms, mp);
fprintf('R  = %.3f m,   l  = %.4f m\n', R, l);
fprintf('Is = %.6f kg*m^2\n', Is);
fprintf('Ip = %.6f kg*m^2\n', Ip);
fprintf('Kp = %.2f,  Kv = %.2f\n', Kp, Kv);
