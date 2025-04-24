function alpha = prindex(x,y)
a = norm(y) + norm(x);
b =0.5;
c = 0;
% if a < 1e-3
%    alpha = 0; 
% else
    alpha = (1-exp(-b*(a-c)))/(1+exp(-b*(a-c)))*15;
%     alpha = 1;
end

