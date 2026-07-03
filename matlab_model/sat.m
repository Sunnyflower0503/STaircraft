function y = sat(x, x_min, x_max)
%SAT  限幅函数 (from BY framework)
    if isscalar(x_min)
        x_min = x_min * ones(size(x));
    else
        x_min = reshape(x_min, size(x));
    end
    if isscalar(x_max)
        x_max = x_max * ones(size(x));
    else
        x_max = reshape(x_max, size(x));
    end
    y = min(max(x, x_min), x_max);
end
