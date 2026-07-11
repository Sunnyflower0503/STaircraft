function dx = tandem_zx_dynamics(t, x, u, param)
%TANDEM_ZX_DYNAMICS  串联翼 eVTOL 6DOF 四元数动力学 (BY 框架兼容)
%
% 对应的 Simulink 子系统:
%   Aerocraft/6DOF (Euler Angles, rep=Quaternion)
%   Aerocraft/F&M (旋翼 + 气动 + 重力)
%
% 状态 x (13):
%   x(1:3)   = p_e      : NED 位置 [m]
%   x(4:6)   = v_e      : NED 速度 [m/s]
%   x(7:10)  = q_eb     : 机体→地球四元数 [qw qx qy qz]'
%   x(11:13) = omega_b  : 机体角速度 [rad/s]
%
% 输入 u (12):
%   u(1:8)  = 油门指令 [0-1] (对应 8 路主旋翼)
%   u(9:10) = 翼尖辅助桨油门 [0-1] (左/右)
%   u(11)   = 左升降副翼 delta_aeL [rad]
%   u(12)   = 右升降副翼 delta_aeR [rad]

if nargin == 3
    param = u;
    if isfield(param, 'actuator_input')
        u = param.actuator_input;
    else
        error('tandem_zx_dynamics:MissingInput', 'Pass u or set param.actuator_input.');
    end
end

if isa(u, 'function_handle')
    u = u(t, x, param);
end

u = u(:);
if numel(u) ~= 12
    error('tandem_zx_dynamics:BadInput', 'u must be [dt1-8; dt_left; dt_right; daeL; daeR] (12).');
end

% --- 提取状态 ---
p_e     = x(1:3);
v_e     = x(4:6);
q_eb    = x(7:10) / norm(x(7:10));      % 归一化四元数
omega_b = x(11:13);

% --- 控制限幅 ---
delta_t  = sat(u(1:8), 0, 1);     % 8 路主旋翼
delta_addL = sat(u(9), 0, 1);      % 左翼尖桨
delta_addR = sat(u(10), 0, 1);     % 右翼尖桨
delta_aeL  = sat(u(11), param.delta_ae_lim(1), param.delta_ae_lim(2));
delta_aeR  = sat(u(12), param.delta_ae_lim(1), param.delta_ae_lim(2));

% --- 四元数 → DCM ---
R_eb = quat2rotm(q_eb');   % body-to-earth
R_be = R_eb';               % earth-to-body

% --- 旋翼力/力矩 (8 主旋翼 + 2 翼尖桨) ---
delta_all = [delta_t; delta_addL; delta_addR];
[f_r, m_r] = tandem_rotor_fm(delta_all, v_e, R_eb, param);

% --- 气动力/力矩 ---
[f_a, m_a] = tandem_aero_fm(v_e, R_eb, omega_b, delta_aeL, delta_aeR, ...
                             delta_t, param);

% --- 重力 ---
f_g_e = [0; 0; param.m * param.g];   % NED: gravity points +z(down)
f_g_b = R_be * f_g_e;

% --- 合力 / 合力矩 (机体坐标系) ---
f_total_b = f_r + f_a + f_g_b;
m_total_b = m_r + m_a;

% --- 地面接触力 ---
if isfield(param, 'ground') && isfield(param.ground, 'enable') && param.ground.enable
    [f_ground_b, m_ground_b] = zx_ground_contact_force(p_e, v_e, R_eb, omega_b, param);
    f_total_b = f_total_b + f_ground_b;
    m_total_b = m_total_b + m_ground_b;
end

% ======= 6DOF 四元数运动方程 =======

% 位置导数: dp_e/dt = v_e
dp_e = v_e;

% 速度导数: dv_e/dt = R_eb * f_total_b / m
dv_e = R_eb * f_total_b / param.m;

% 四元数导数: dq/dt = 0.5 * Omega(omega_b) * q
Omega = [0,          -omega_b(1), -omega_b(2), -omega_b(3);
         omega_b(1),  0,           omega_b(3), -omega_b(2);
         omega_b(2), -omega_b(3),  0,           omega_b(1);
         omega_b(3),  omega_b(2), -omega_b(1),  0];
dq = 0.5 * Omega * q_eb;

% 角速度导数: J * domega/dt = m - omega × (J * omega)
domega_b = param.J \ (m_total_b - cross(omega_b, param.J * omega_b));

dx = [dp_e; dv_e; dq; domega_b];
end
