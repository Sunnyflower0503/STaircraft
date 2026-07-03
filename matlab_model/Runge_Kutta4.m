%书籍：常用数值算法及其matlab实现
%第10章 常微分方程初值问题的数值解法,例10.14使用
%四阶龙格库塔方法
function [t,z,zdot] = Runge_Kutta4(fun, t, X0)
    z = zeros(length(X0), length(t));
    z(:,1) = X0(:);
    for i = 1:length(t)-1
        h = t(i+1) - t(i);
        K1 = feval(fun, t(i),       z(:,i));
        K2 = feval(fun, t(i)+h/2,   z(:,i)+h/2*K1);
        K3 = feval(fun, t(i)+h/2,   z(:,i)+h/2*K2);
        K4 = feval(fun, t(i)+h,     z(:,i)+h*K3);
        z(:,i+1) = z(:,i) + h/6*(K1 + 2*K2 + 2*K3 + K4);
        zdot(:,i) = K1;
    end
end
