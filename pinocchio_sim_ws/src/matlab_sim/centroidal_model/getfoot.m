function foot = getfoot(U, X)
global 	I m g c
    % state
    theta = X(1:3); % ZYX eular angles
    p = X(4:6); % 
    omega = X(7:9);
    v = X(10:12);
    p1 = X(13:15);
    p2 = X(16:18);
    
    % input
    f1 = U(1:3);
    f2 = U(4:6);
    dp1 = U(7:9);
    dp2 = U(10:12);
    
    z = theta(1); y = theta(2); x = theta(3);
    % ZYX Eular angle to angle velocity (inverse)
    T = [cos(z)*sin(y)/cos(y), sin(y)*sin(z)/cos(y), 1;
         -sin(z),              cos(z),               0;
         cos(z)/cos(y),        sin(z)/cos(y),        0];
    % Rotation matrix
    Rz = [cos(z), -sin(z), 0;
          sin(z), cos(z),  0;
          0,      0,       1];
    Ry = [cos(y),  0, sin(y);
          0,       1, 0;
          -sin(y), 0, cos(y)];
    Rx = [1, 0, 0;
          0, cos(x), -sin(x);
          0, sin(x), cos(x)];
    R = Rz*Ry*Rx;
        R1x = R(1,1);R1y = R(1,2);R1z = R(1,3);
    R2x = R(2,1);R2y = R(2,2);R2z = R(2,3);
    R3x = R(3,1);R3y = R(3,2);R3z = R(3,3);
    dtheta = T*omega;
    dp = R*v;
    % 重力作用于脚上，垂直向上 
    N = [(245*(R1x*R2y*R1z - R2x*R1y*R1z))/(121*(R1x*R2y*R3z - R1x*R3y*R2z - R2x*R1y*R3z + R2x*R3y*R1z + R3x*R1y*R2z - R3x*R2y*R1z));
          (245*R2z*(R1x*R2y - R2x*R1y))/(121*(R1x*R2y*R3z - R1x*R3y*R2z - R2x*R1y*R3z + R2x*R3y*R1z + R3x*R1y*R2z - R3x*R2y*R1z));
          (49*(121*R1x*R2y*R3z - 96*R1x*R3y*R2z - 121*R2x*R1y*R3z + 96*R2x*R3y*R1z + 96*R3x*R1y*R2z - 96*R3x*R2y*R1z))/(605*(R1x*R2y*R3z - R1x*R3y*R2z - R2x*R1y*R3z + R2x*R3y*R1z + R3x*R1y*R2z - R3x*R2y*R1z))];
    domega =  I\(-cross(omega, I*omega) + cross(p1, f1) + cross(p2, f2) + cross([0;0;-c/2], R\N));
    dv = ((f1 + f2) + R\(N + m*g))/m;
    dX = [dtheta; dp ; domega; dv; dp1; dp2];
    dfoot = dv + generate_cross_matrix(domega)*[0;0;-c/2]
    foot = p + R * [0;0;-c/2];
end