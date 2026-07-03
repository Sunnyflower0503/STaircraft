function [f_r, m_r] = tandem_rotor_fm(delta_t, Ve, DCM_be, param)
%TANDEM_ROTOR_FM  8 旋翼 + 2 翼尖辅助桨合力/合力矩 (对应 Thrust&M in body axis)
%
% 输入:
%   delta_t : 10 路油门 [0-1]，也兼容只传 8 路主旋翼
%             delta_t(1:8) = 8 路主旋翼
%             delta_t(9)   = 左翼尖桨
%             delta_t(10)  = 右翼尖桨
%   Ve      : NED 地固系速度 [m/s]
%   DCM_be  : body-to-earth 旋转矩阵
%   param   : 参数结构体
%
% 输出:
%   f_r : 机体坐标系合力 [N]
%   m_r : 机体坐标系合力矩 [Nm]

delta_t = delta_t(:);
if numel(delta_t) == 8
    delta_t = [delta_t; 0; 0];
elseif numel(delta_t) ~= 10
    error('tandem_rotor_fm:BadInput', 'delta_t must have 8 main rotors or 10 total propulsion channels.');
end

DCM_eb = DCM_be';
Va_b = DCM_eb * (Ve - param.wind(:));  % 机体空速
Va = norm(Va_b);

if Va < 1e-6
    alpha = 0; beta = 0;
else
    alpha = atan2(Va_b(3), Va_b(1));
    beta = atan2(Va_b(2), hypot(Va_b(1), Va_b(3)));
end

f_r = zeros(3, 1);
m_r = zeros(3, 1);

%% === 8 路主旋翼 ===
if isfield(param, 'rotor_throttle_min')
    main_throttle_min = param.rotor_throttle_min;
else
    main_throttle_min = 0;
end

for i = 1:8
    dt_i = sat(delta_t(i), main_throttle_min, 1);

    [T_i, M_i, ~, ~] = tandem_rotor_thrust(dt_i, alpha, beta, Va, ...
                                            param.prop_spin(i), param, ...
                                            param.prop_angle(i));

    if T_i <= 0, continue; end

    % 推力矢量 (机体坐标系)
    a = param.prop_angle(i);
    ca = cos(a); sa = sin(a);
    f_i = T_i * [ca; 0; -sa];  % Tx=T*cos(a), Tz=-T*sin(a)
    f_r = f_r + f_i;

    % 位置交叉积 → 力矩
    r_i = param.prop_pos(i, :)';
    m_r = m_r + cross(r_i, f_i);

    % 反扭矩
    m_r = m_r + M_i * [ca; 0; sa];
end

%% === 2 路翼尖辅助桨 ===
dt_left  = delta_t(9);
dt_right = delta_t(10);

[T_add, M_add] = tandem_addprop_fm(dt_left, dt_right, param);

% 翼尖桨推力
f_r = f_r + [0; 0; T_add(1) + T_add(2)];

% 翼尖桨力矩
m_r = m_r + M_add;
end
