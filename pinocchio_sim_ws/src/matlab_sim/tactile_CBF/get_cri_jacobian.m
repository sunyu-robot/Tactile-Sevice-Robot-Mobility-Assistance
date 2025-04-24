function jacobian = get_cri_jacobian(k,q_real)
    jacobian = zeros(2,3);
    switch k
        case {1,2,3,4}
            jacobian = [1,0,0;0,0,0];
        case {5,6,7,8,9}
            jacobian = [1, -sin(q_real(2))*0.25*(k-5),0;0,cos(q_real(2))*0.25*(k-5),0];
        case {10,11,12,13}
            jacobian = [1, -0.25*(k-9)*sin(q_real(2) + q_real(3)) - 0.25*(k-9)*sin(q_real(2)),...
                -sin(q_real(2) + q_real(3)); 0,   0.25*(k-9)*cos(q_real(2) + q_real(3)) +...
                1*cos(q_real(2)),  0.25*(k-9)*cos(q_real(2) + q_real(3))];
        otherwise
            warning('错误关键点编号')
end