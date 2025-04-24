function center = forward_critical_point(k,q_real,pos)
    center = [0;0];
    switch k
        case 1
            center(1) = q_real(1) + 0.5;
            center(2) = 0.2;
        case 2
            center(1) = q_real(1) + 0.5;
            center(2) = -0.2;
        case 3
            center(1) = q_real(1) - 0.5;
            center(2) = 0.2;        
        case 4
            center(1) = q_real(1) - 0.5;
            center(2) = -0.2;
        case {5,6,7,8,9}
            center(1) = 0.25*(9-k)*q_real(1)+0.25*(k-5)*pos(1);
            center(2) = 0.25*(k-5)*(pos(2));
        case {10,11,12,13}
            center(1) = 0.25*(k-9)*pos(3)+0.25*(13-k)*pos(1);
            center(2) = 0.25*(k-9)*pos(4)+0.25*(13-k)*pos(2);
        otherwise
            warning('错误关键点编号')
    end

end