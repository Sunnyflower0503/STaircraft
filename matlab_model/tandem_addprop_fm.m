function [T_add, M_add] =  tandem_addprop_fm(dt_left, dt_right, param)
%TANDEM_ADDPROP_FM  翼尖辅助桨推力/力矩 (对应 Addprop Left/Right)
%
% 当前模型来自 15.8 V 静力测试数据，先由油门查表/拟合电功率 P_e(dt)，
% 再由电功率查表/拟合静推力 T(P_e)。T_left/T_right 为正的推力幅值。
%
% 机体系采用 +X=前, +Y=右, +Z=下；翼尖桨气动力沿 -Zb。
% T_add 输出保持历史接口约定为负值，供 tandem_rotor_fm 累加到 Fz。
%
% param.addprop_moment_mode:
%   "fixed_wing" : 差动翼尖桨产生滚转力矩 Mx
%   "rotor_yaw"  : 差动翼尖桨等效为旋翼模式偏航力矩 Mz
%   "geometry"   : 采用 r × F 几何力矩，兼容旧模型
%
% 输入:
%   dt_left  : 左翼尖桨油门 [0-1]
%   dt_right : 右翼尖桨油门 [0-1]
%   param    : 参数结构体
%
% 输出:
%   T_add : [T_left; T_right] 推力接口量 [N]，负号表示沿 -Zb
%   M_add : 合力矩 [Mx; My; Mz] [Nm]

[T_left, ~] = addprop_static_thrust(dt_left, param);
[T_right, ~] = addprop_static_thrust(dt_right, param);

mode = "fixed_wing";
if isfield(param, 'addprop_moment_mode')
    mode = string(param.addprop_moment_mode);
end

switch lower(mode)
    case "fixed_wing"
        M_add = [param.addprop_y * (T_left - T_right); 0; 0];
    case {"rotor_yaw", "rotor"}
        yaw_arm = get_optional_scalar(param, 'addprop_rotor_yaw_arm', param.addprop_y);
        M_add = [0; 0; yaw_arm * (T_left - T_right)];
    case "geometry"
        M_left = cross([param.addprop_x; -param.addprop_y; 0], [0; 0; -T_left]);
        M_right = cross([param.addprop_x; param.addprop_y; 0], [0; 0; -T_right]);
        M_add = M_left + M_right;
    otherwise
        error('tandem_addprop_fm:InvalidMomentMode', ...
            'Unknown addprop_moment_mode: %s', mode);
end

T_add = [-T_left; -T_right];
end

function [T, P_e] = addprop_static_thrust(dt, param)
dt = min(max(dt, 0), 1);

if isfield(param, 'addprop_throttle_bp') && isfield(param, 'addprop_power_table') ...
        && isfield(param, 'addprop_thrust_power_bp') && isfield(param, 'addprop_thrust_table')
    P_e = interp1(param.addprop_throttle_bp, param.addprop_power_table, dt, 'linear', 'extrap');
    P_e = min(max(P_e, 0), max(param.addprop_power_table));
    T = interp1(param.addprop_thrust_power_bp, param.addprop_thrust_table, P_e, 'linear', 'extrap');
    T = max(T, 0);
    return;
end

if isfield(param, 'addprop_power_coef') && isfield(param, 'addprop_thrust_power_coef')
    P_e = max(polyval(param.addprop_power_coef, dt), 0);
    T = max(polyval(param.addprop_thrust_power_coef, P_e), 0);
    return;
end

n = polyval([-3424 12810 -4], dt) / 60;
if n > 0
    CT = polyval([5.87e-8 -1.61e-5 0.0014 0.1137], n);
    T = max(CT * param.rho * n^2 * param.addprop_D^4, 0);
else
    T = 0;
end
P_e = NaN;
end

function value = get_optional_scalar(param, field_name, default_value)
value = default_value;
if isfield(param, field_name)
    candidate = param.(field_name);
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end
end
