function [T_add, M_add] =  tandem_addprop_fm(dt_left, dt_right, param)
%TANDEM_ADDPROP_FM  翼尖辅助桨推力/力矩 (对应 Addprop Left/Right)
%
% C 代码 (Tandem_zx_trans6_addprop):
%   n = polyval([-3424 12810 -4], dt) / 60  [rev/s]
%   T_add = polyval([5.87e-8 -1.61e-5 1.4e-3 0.1137], n) * rho * n^2 * D^4
%   D_add = 0.127 m
%
% 左桨力矩 (chart_135):
%   Mx_left =  T_add * prop_y
%   My_left =  T_add * prop_x
%   Mz_left =  0
% 右桨力矩 (chart_151):
%   Mx_right = -T_add * prop_y
%   My_right =  T_add * prop_x
%   Mz_right =  0
%
% 输入:
%   dt_left  : 左翼尖桨油门 [0-1]
%   dt_right : 右翼尖桨油门 [0-1]
%   param    : 参数结构体
%
% 输出:
%   T_add : [T_left; T_right] 推力 [N]
%   M_add : 合力矩 [Mx; My; Mz] [Nm]

% --- 左翼尖桨推力 ---
n_left = polyval([-3424 12810 -4], dt_left) / 60;  % rev/s
if n_left > 0
    CT_left = polyval([5.87e-8 -1.61e-5 0.0014 0.1137], n_left);
    T_left = CT_left * param.rho * n_left^2 * param.addprop_D^4;
else
    T_left = 0;
end    

% --- 右翼尖桨推力 ---
n_right = polyval([-3424 12810 -4], dt_right) / 60;
if n_right > 0
    CT_right = polyval([5.87e-8 -1.61e-5 0.0014 0.1137], n_right);
    T_right = CT_right * param.rho * n_right^2 * param.addprop_D^4;
else
    T_right = 0;
end

% --- 力矩累计 ---
% 机体坐标系: +X=前, +Y=右, +Z=下
% 推力方向均为 -z (向上, 安装角=0)
%
% 左桨 r = [addprop_x, -addprop_y, 0]  (-Y)
%   M_left  = r × [0,0,-T_left]  = [+T_left *addprop_y,  +T_left *addprop_x,  0]
% 右桨 r = [addprop_x, +addprop_y, 0]  (+Y)
%   M_right = r × [0,0,-T_right] = [-T_right*addprop_y,  +T_right*addprop_x,  0]

% 左桨力矩
Mx_left  =  T_left * param.addprop_y;
My_left  =  T_left * param.addprop_x;
Mz_left  =  0;

% 右桨力矩
Mx_right = -T_right * param.addprop_y;
My_right =  T_right * param.addprop_x;
Mz_right =  0;

% Optional rotor-mode yaw authority.  The original Simulink-matched addprop
% geometry has zero yaw moment.  When addprop_yaw_coeff is provided by the
% controller parameter set, differential wingtip-prop thrust is interpreted
% as an equivalent propeller reaction yaw moment for rotor-mode control tests.
if isfield(param, 'addprop_yaw_coeff') && abs(param.addprop_yaw_coeff) > 0
    Mz_left  =  param.addprop_yaw_coeff * T_left;
    Mz_right = -param.addprop_yaw_coeff * T_right;
end

T_add = [-T_left; -T_right];
M_add = [Mx_left + Mx_right; My_left + My_right; Mz_left + Mz_right];
end
