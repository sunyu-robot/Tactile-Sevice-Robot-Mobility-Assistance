clear;clc;
syms q1 q2 q0 l1 l2
%% forward kinematics
% x = q0 + l1*cos(q1) + l2*cos(q1+q2);
% y = l1*sin(q1) + l2*sin(q1+q2);
x = q0 + l1*cos(q1) + 0.2*l2*cos(q1+q2);
y = l1*sin(q1) + 0.2*l2*sin(q1+q2);
x1 = q0 + 0.8*l1*cos(q1);
y1 = 0.8*l1*sin(q1);

T1 = [1,0,q0;
      0,1,0;
      0,0,1];
T2 = [cos(q1),sin(q1),q0;
      -sin(q1),cos(q1),0;
      0,0,1];
  
T3 = [cos(q1+q2),sin(q1+q2),x1;
      -sin(q1+q2),cos(q1+q2),y1;
      0,0,1];
q = [q0;q1;q2];
R2 = T2(1:2,1:2);
R3 = T3(1:2,1:2);
dR2 = [diff(R2,q0);diff(R2,q1);diff(R2,q2)]; 
dR3 = [diff(R3,q0);diff(R3,q1);diff(R3,q2)]; 
R3l = R3 *[0;0.5];
a = dR3*[0;0.5];
JR = [diff(R3l(1),q0),diff(R3l(1),q1),diff(R3l(1),q2);
    diff(R3l(2),q0),diff(R3l(2),q1),diff(R3l(2),q2)];

J = [diff(x,q0),diff(x,q1),diff(x,q2);
    diff(y,q0),diff(y,q1),diff(y,q2)];

J2 = [diff(x1,q0),diff(x1,q1),diff(x1,q2);
    diff(y1,q0),diff(y1,q1),diff(y1,q2)];

jacobi = subs(J,[l1,l2],[1,1]);
jacobi2 = subs(J2,[l1,l2],[1,1]);

% %% cri jacobian
% x = q0 + l1*cos(q1) + 0.2*l2*cos(q1+q2);
% y = l1*sin(q1) + 0.2*l2*sin(q1+q2);

