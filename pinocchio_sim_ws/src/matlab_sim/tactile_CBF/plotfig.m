clear;clc;
b =2;
c = 0;
fun = @(a) (1-exp(-b*(a-c)))/(1+exp(-b*(a-c)));

% dfun =  @(a) exp((log(20)-log(10))*a/6);
% anss = fzero(fun,1);
x = 0:0.01:3;


for i=1:size(x,2)
    y(i) = fun(x(i));
end
plot(x,y,'-','linewidth',2),title('tanh'),xlabel('x两阶导数'),ylabel('优先级系数\alpha');