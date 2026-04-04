function p = get_params()
% get_params.m
% MATLAB Function 블록 안에서 파라미터를 불러오는 헬퍼 함수

p.Ms    = 3;
p.mp    = 2;
p.R     = 0.2;
p.l     = 0.075;
p.g     = 9.81;
p.Is    = (2/3) * 3 * 0.2^2;
p.Ip    = (1/3) * 2 * 0.075^2;
p.Kp    = 1;
p.Kv    = 1;
p.u_max = 2.5;
end