%离散pid （增量式）
function u = discretePID(e, e_prev, e_lastprev, lastu)
kp = 200;
ki = 2;
kd = 10;

deltau = kp*(e-e_prev) + ki*(e) + kd*(e-2*e_prev+e_lastprev);
u = lastu + deltau;
end
