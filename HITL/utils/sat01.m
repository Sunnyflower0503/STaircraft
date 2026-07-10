function y = sat01(x)
%SAT01 Saturate values to [0, 1].
y = min(max(x, 0), 1);
end
