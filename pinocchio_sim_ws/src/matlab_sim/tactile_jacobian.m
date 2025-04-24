% Calculate Jacobian matrix of tactile contact on robot manipulator
clear;

syms qb q0 q1 q2 q3 q4 q5 q6 
syms    d1       d4 d5 d6
syms       a2 a3     
syms  xb0  yb0  zb0
syms  xwb  ywb
syms  xa  ya  za

syms  Jv  Jw  
%%*********** Standard DH

Awb =  [cos(qb)  -sin(qb)   0    xwb
        sin(qb)   cos(qb)   0    ywb  
             0    0         1    0
             0    0         0    1];  
%%*********** The installation matrix of the manipulator relative to the mecanum 
syms  a  b  c
% left
% Ab0 =  [1  0   0    a
%         0  0  1    b 
%         0  -1  0    c
%         0  0   0    1]; 

%right
Ab0 =  [-1  0   0    a
        0  0  -1    b 
        0  -1  0    c
        0  0   0    1];
%*********** Forward kinematics          
A01 = [cos(q1)   0    sin(q1)   0
       sin(q1)   0   -cos(q1)   0 
            0    1         0   d1
            0    0         0    1];
        
A12 = [cos(q2)  -sin(q2)   0    a2*cos(q2)
       sin(q2)   cos(q2)   0    a2*sin(q2)
            0    0         1    0
            0    0         0    1];

A23 = [cos(q3)  -sin(q3)   0    a3*cos(q3)
       sin(q3)   cos(q3)   0    a3*sin(q3)
            0    0         1    0
            0    0         0    1];

A34 = [cos(q4)   0    sin(q4)   0
       sin(q4)   0   -cos(q4)   0 
            0    1         0   d4
            0    0         0    1];
 
A45 = [cos(q5)   0   -sin(q5)   0
       sin(q5)   0    cos(q5)   0 
            0   -1         0   d5
            0    0         0    1];
        
A56 = [cos(q6)  -sin(q6)   0    0
       sin(q6)   cos(q6)   0    0
            0    0         1   d6
            0    0         0    1];


Aw0 = simplify(Awb*Ab0);
Aw1 = simplify(Awb*Ab0*A01); 
Aw2 = simplify(Awb*Ab0*A01*A12); 
Aw3 = simplify(Awb*Ab0*A01*A12*A23); 
Aw4 = simplify(Awb*Ab0*A01*A12*A23*A34);    
Aw5 = simplify(Awb*Ab0*A01*A12*A23*A34*A45); 
Aw6 = simplify(Awb*Ab0*A01*A12*A23*A34*A45*A56); 


%% Whole-body Jacobian - The velocity mapping of the end-effector from the joint framework to the Cartesian framework

Jmv= [diff(Aw6(1,4),xwb)  diff(Aw6(1,4),ywb)  diff(Aw6(1,4),qb) diff(Aw6(1,4),q1)  diff(Aw6(1,4),q2)  diff(Aw6(1,4),q3)  diff(Aw6(1,4),q4)  diff(Aw6(1,4),q5)  diff(Aw6(1,4),q6)
      diff(Aw6(2,4),xwb)  diff(Aw6(2,4),ywb)  diff(Aw6(2,4),qb) diff(Aw6(2,4),q1)  diff(Aw6(2,4),q2)  diff(Aw6(2,4),q3)  diff(Aw6(2,4),q4)  diff(Aw6(2,4),q5)  diff(Aw6(2,4),q6)
      diff(Aw6(3,4),xwb)  diff(Aw6(3,4),ywb)  diff(Aw6(3,4),qb) diff(Aw6(3,4),q1)  diff(Aw6(3,4),q2)  diff(Aw6(3,4),q3)  diff(Aw6(3,4),q4)  diff(Aw6(3,4),q5)  diff(Aw6(3,4),q6)];

jacobi_3_9_L = subs(Jmv,[a, b, c, d1, d4, d5, d6, a2, a3],[0.460, 0.146, 1.300,  0.1807, 0.17415, 0.11985, 0.11655, -0.4784, -0.36]);
jacobi_3_15_L = [jacobi_3_9_L , zeros(3,6)];

