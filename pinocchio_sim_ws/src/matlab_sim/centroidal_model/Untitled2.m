% 定义全局变量
global m c g

% 定义符号变量
syms p1x p1y p1z p2x p2y p2z f1x f1y f1z f2x f2y f2z Nx Ny Nz omegax omegay omegaz real
syms R1x R1y R1z R2x R2y R2z R3x R3y R3z real
syms vx vy vz

% 定义向量和矩阵
p1 = [p1x; p1y; p1z];
p2 = [p2x; p2y; p2z];
f1 = [f1x; f1y; f1z];
f2 = [f2x; f2y; f2z]; 
N = [Nx; Ny; Nz]; 
v = [vx;vy;vz];
omega = [omegax; omegay; omegaz];
R = [R1x, R1y, R1z;
     R2x, R2y, R2z;
     R3x, R3y, R3z];

% 定义常量
a = 1; 
b = 1;
c = 5;
m = 1;
I = (a^2 + b^2) * m * 12 * eye(3); % 惯性矩阵 (Z 轴)
g = [0; 0; -9.8];
L = [0; 0; -c/2];
% 计算角加速度 domega
domega = I \ (-cross(omega, I * omega) + cross([0; 0; -c/2], R \ N));
dv = R \(N + m*g)/m;
% 计算力 F
F = R \ N + R \ (m * g) + m * cross(domega, [0; 0; -c/2]);
% eqa = generate_cross_matrix(omega)*v + dv + generate_cross_matrix(omega)*generate_cross_matrix(omega)*L + generate_cross_matrix(domega)*L;
eqa = dv + generate_cross_matrix(domega)*L;
% 初始旋转矩阵
x = 0;
R_init = [1, 0, 0;
          0, cos(x), -sin(x);
          0, sin(x), cos(x)];

% 求解 N
N_sol = solve(eqa == 0, [Nx, Ny, Nz]);

% 替换 R 为 R_init
N_l = subs(N_sol, [R1x, R1y, R1z, R2x, R2y, R2z, R3x, R3y, R3z], R_init(:)');

% 显示 N_l 的结果
disp(N_l);
disp('Nx:');
disp(N_sol.Nx);
disp('Ny:');
disp(N_sol.Ny);
disp('Nz:');
disp(N_sol.Nz);
disp('Nx:');
disp(N_l.Nx);
disp('Ny:');
disp(N_l.Ny);
disp('Nz:');
disp(N_l.Nz);