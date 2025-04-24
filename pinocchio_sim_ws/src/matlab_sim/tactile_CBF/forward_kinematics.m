function pos = forward_kinematics(q, L1, L2)
% q1 and q2 are the joint angles in radians
% L1 and L2 are the lengths of the arm segments
q1 = q(1); q2 = q(2);
x1 = L1*cos(q1);
y1 = L1*sin(q1);
x2 = L1*cos(q1) + L2*cos(q1+q2);
y2 = L1*sin(q1) + L2*sin(q1+q2);
pos = [x1,y1,x2,y2];
end