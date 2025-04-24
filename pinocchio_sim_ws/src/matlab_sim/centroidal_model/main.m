%% nmpc
% dependencies : casadi
% model : centroidal dynamics
import casadi.*
clear;clc;
format long
%------------------------------config-----------------------------------%
h = 0.04; % step
n = 5; % whole time
N = n/h;  % whole step
t = zeros(1,N); % time vec

% centroidal dynamcis
global I m g c mu
% Cuboid assumption
a = 1; b = 1; c = 5;
m = 1;
I = (a^2 + b^2)*m*12; % inertial matrix (z axis)
g  = [0;0;-9.8];
mu = 0.3;
% state (theta p omega v p_i \in R^{18})
dq = zeros(18,N);
q = zeros(18,N);
% input
u = zeros(12,N);
% initial state
q0 = zeros(18,1);
q0(1:3) = [0;0;0.1];
R0 = getR(q0(1:3));
q0(4:6) = - R0*[0; 0; -c/2];
% 接触的初始位置
q0(13:15) = [0; b/2; 0];
q0(16:18) = [0; -b/2; 0];
q(:,1) = q0;
% desired state
qd = zeros(18,1);
qd(4:6) = [0; 0; c/2];
foot = zeros(3, N);
% NMPC
p_h = 20;
Q = diag([1000*ones(1,3),400*ones(1,3),100*ones(1,3), 40*ones(1,3), 40*ones(1,6)]); % theta p omega v p_i
R = diag([1e-3 * ones(1,6), 1e-2 * ones(1,6)]); % lambda 
%% Loop begin
for i = 1:N-1
    % control input
    u(:,i) = centroidalNMPC(Q, R, q(:,i), qd, p_h, h);
%     foot(:,i) = getfoot(u(:,i), q(:, i))
    % dynamics (rk4)
    k1 = centroidalDynamics(u(:,i), q(:, i));
    k2 = centroidalDynamics(u(:,i), q(:, i) + 0.5 * h * k1);
    k3 = centroidalDynamics(u(:,i), q(:, i) + 0.5 * h * k2);
    k4 = centroidalDynamics(u(:,i), q(:, i) + h * k3);
    q(:, i+1) = q(:, i) + (h/6) * (k1 + 2 * k2 + 2 * k3 + k4);
    
    % update time
    t(i+1) = t(i)+h;
end
%% draw figure
% euler angle fig
figure(1)
plot(t,q(1,:),'-','linewidth',2);hold on;
plot(t,q(2,:),'-','linewidth',2);hold on;
plot(t,q(3,:),'-','linewidth',2);hold on;
title('Euler angle');grid on;
legend('z','y','x');
% position fig
figure(2)
plot(t,q(4,:),'-','linewidth',2);hold on;
plot(t,q(5,:),'-','linewidth',2);hold on;
plot(t,q(6,:),'-','linewidth',2);hold on;
title('COM position');grid on;
legend('x','y','z');
% anime
% position fig
% video = VideoWriter('cartpole.mp4', 'MPEG-4');
% video.FrameRate = 1/h; 
% open(video);

figure(3)
for i = 1:N
    % pole
    plot([q(1,i) q(1,i)+l*sin(q(2,i))],[0 l*cos(q(2,i))],'LineWidth',4);
    % cart
    width = 0.6;height = 0.3;
    rectangle('Position', [q(1,i)-width/2,-height/2, width, height], 'FaceColor', 'none', 'EdgeColor', 'k','LineWidth',2);
    hold on;
    % ball
    radius = 60;
    scatter(q(1,i)+l*sin(q(2,i)), l*cos(q(2,i)), radius, 'filled', 'r');
    hold off;
    xlim([-2 2]); ylim([-2 2]);  
    xlabel('X'); ylabel('Y');  
    grid on; 
%     frame = getframe(gcf);
%     writeVideo(video, frame);
end
% close(video);