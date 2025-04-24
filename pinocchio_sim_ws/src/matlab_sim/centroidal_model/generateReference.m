function x_ref = generateReference(x,x_des,h)
    num_variable_ = size(x, 1);
    x_ref = zeros(num_variable_, h);
    for i = 1 : num_variable_
        x_ref(i,:) = linspace(x(i), x_des(i), h);
    end
end