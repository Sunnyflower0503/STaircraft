function param = init_param_zx()
%INIT_PARAM_ZX  Tandem ZX eVTOL 模型参数初始化 (BY 框架风格)
% 参数来源: Tandem_zx_trans6_grt_rtw/Tandem_zx_trans6_data.c

%% === 物理常数 ===
param.g = 9.8;                         % 重力加速度 [m/s^2]
param.rho = 1.225;                     % 空气密度 [kg/m^3]
param.D2R = pi/180; param.R2D = 180/pi;

%% === 质量和惯性 ===
param.m = 3.2;                         % 总质量 [kg]
Ixx=0.3236731; Iyy=0.45521417; Izz=0.616930086; Ixz=0.167493906;
param.J = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];  % 惯性矩阵
param.invJ = inv(param.J);

%% === 翼面几何 ===
param.S = 0.732;                       % 翼面积 [m^2]
param.b = 1.2;                         % 参考翼展 [m]
param.MAC = 0.3;                       % 平均气动弦长 [m]
param.S_slip = [0.03048 0.06858 0.06858 0.03048 0.03048 0.06858 0.06858 0.03048];
param.S_slip_y = [-0.58 -0.175 0.175 0.58 -0.58 -0.175 0.175 0.58];
param.S_free = param.S - sum(param.S_slip);
param.slipstream_enable = true;        % 是否考虑 8 个主桨滑流区气动力
param.slipstream_ff_enable = true;     % 是否使用滑流区 f_f 修正系数；false 时 f_f=1
param.alpha_slip_switch = 1;
param.DP_slip_switch = 1;

%% === 8 旋翼布局 ===
param.prop_D = 0.2032;                 % 螺旋桨直径 [m]
param.prop_pos = [...
    0.371 -0.5845  0.175;
    0.371 -0.175   0.175;
    0.371  0.175   0.175;
    0.371  0.5845  0.175;
   -0.329 -0.5845 -0.175;
   -0.329 -0.175  -0.175;
   -0.329  0.175  -0.175;
   -0.329  0.5845 -0.175];
param.prop_angle = zeros(8,1);         % 安装角=0
param.prop_spin = [-1 -1 -1 -1 1 1 1 1]; % front=clockwise, behind=anticlockwise

%% === 翼尖辅助桨 (Addprop Left / Addprop Right) ===
param.addprop_x = 0.3;                 % 辅助桨 x 位置 [m]
param.addprop_y = 0.65;                % 辅助桨 y 位置 [m]
param.addprop_D = 0.127;               % 辅助桨直径 [m] (from C code: D_add=0.127)

%% === 气动系数 (from C code data.c struct_oB3eqNsKceXibtnfQbk3QB) ===
a = struct();
a.CD0 = 0; a.CDdelta_aeL = 0.01905225; a.CDdelta_aeR = 0.01905225;
a.CYbeta = -0.245;
a.CLdelta_aeL = 0.2066238; a.CLdelta_aeR = 0.2066238;
a.CLq = 6.27; a.CLalpdot = 0.627;
a.Cldelta_aeL = 0.054435; a.Cldelta_aeR = -0.054435;
a.Cmdelta_aeL = -0.33234; a.Cmdelta_aeR = -0.33234;
a.Cmq = -11.019236; a.Cmalpdot = -4.4076944;
a.Cndelta_aeL = -0.0051; a.Cndelta_aeR = 0.0051;
param.aero = a;

%% === CFD 表格 ===
param.bigalpha_arr = [-6 -4 -2 0 2 4 6 8 10 12 14 15 20 25 30 35 40 45 50]*param.D2R;
param.alpha_arr = [-4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9]*param.D2R;
param.aero.CD_arr = [0.05703 0.05363 0.05478 0.06171 0.07262 0.08715 0.10403 0.12350 0.14423 0.16806 0.19569 0.21104 0.33602 0.44497 0.66637 0.89926 1.11543 1.28046 1.48742];
param.aero.CL_arr = [0.13177 0.23946 0.35383 0.45618 0.56180 0.66207 0.76109 0.85925 0.95104 1.03631 1.11760 1.15681 1.33557 1.49766 1.64227 1.75166 1.78420 1.73537 1.60000];
param.aero.Cm_arr = [0.105 0.1162 0.1037 0.0368 0.0224 0.0063 -0.0137 -0.0442 -0.0922 -0.1522 -0.2226 -0.2787 -0.3129 -0.3380 -0.3631 -0.3882 -0.4133 -0.4385 -0.4829];
param.aero.CYp_arr = [-0.099517 -0.073394 -0.047124 -0.020741 0.005724 0.032238 0.058769 0.085285 0.111754 0.138142 0.164419 0.190552 0.216509 0.242258];
param.aero.CYr_arr = [0.250408 0.255041 0.258759 0.261558 0.263434 0.264386 0.264412 0.263512 0.261687 0.258939 0.255273 0.250691 0.245200 0.238807];
param.aero.Clbeta_arr = [-0.068413 -0.077799 -0.087178 -0.096538 -0.105868 -0.115156 -0.124393 -0.133565 -0.142662 -0.151673 -0.160586 -0.169392 -0.178079 -0.186637];
param.aero.Clp_arr = [-0.35762 -0.354978 -0.352067 -0.348891 -0.345458 -0.341774 -0.337848 -0.333687 -0.3293 -0.324696 -0.319885 -0.314875 -0.309678 -0.304303];
param.aero.Clr_arr = [0.116171 0.130706 0.145105 0.159353 0.17344 0.187353 0.201079 0.214609 0.22793 0.241034 0.253909 0.266546 0.278938 0.291074];
param.aero.Cnbeta_arr = [0.094369 0.09441 0.094779 0.095474 0.096496 0.097843 0.099513 0.101505 0.103816 0.106442 0.109381 0.11263 0.116184 0.120039];
param.aero.Cnp_arr = [0.059446 0.054013 0.048466 0.042799 0.037006 0.03108 0.025015 0.018808 0.012452 0.005943 -0.000722 -0.007548 -0.014536 -0.02169];
param.aero.Cnr_arr = [-0.103778 -0.106942 -0.110236 -0.113654 -0.117188 -0.120832 -0.124577 -0.128416 -0.13234 -0.136341 -0.140409 -0.144535 -0.148709 -0.15292];

%% === 全角度气动补全 (Quan et al. flat-plate/post-stall extension) ===
param.aero_extend_mode = 'flat_plate';
param.aero_fullalpha_arr = (-180:1:180) * param.D2R;
param.aero_extension_blend = 10 * param.D2R;
param.aero.flat_plate = zx_fit_flat_plate_params(param);
[param.aero.CD_full_arr, param.aero.CL_full_arr] = zx_build_full_aero_tables(param);

%% === 初始状态 (来自 InitData in init_HIL_llf.m) ===
param.InitData.Xe    = [0; 0; -1.8];          % NED 初始位置 [m]
param.InitData.Vb    = [0.1; 0; 0];            % 机体初始速度 [m/s]
param.InitData.Euler = [0; 30*param.D2R; 0];   % [roll; pitch; yaw] [rad]
param.InitData.pqr   = [0; 0; 0];              % 初始角速度 [rad/s]

% 兼容旧字段
param.amsl0 = 1.8;

%% === 推力范围 ===
param.thrust_min = 0;
param.thrust_max = 25;                 % 单旋翼最大推力 [N]
param.rotor_throttle_min = 0;           % 物理油门下限；0 表示螺旋桨可完全停转

%% === 舵面 ===
param.delta_ae_lim = [-30*param.D2R, 30*param.D2R];

%% === 风速 ===
param.wind = [0; 0; 0];

%% === 地面接触模型 ===
% NED 坐标下 z 向下，地面平面默认 z=0。默认关闭以保持已有空中仿真结果不变。
param.ground.enable = false;
param.ground.z = 0;                    % 地面高度 [m]
param.ground.tol = 1e-4;               % 接触判定容差 [m]
param.ground.k = 700;                  % 穿地修正弹簧 [N/m]
param.ground.c = 70;                   % 法向阻尼 [N/(m/s)]
param.ground.mu = 0.55;                % 地面摩擦系数
param.ground.xy_damping = 35;          % 低速滑行阻尼 [N/(m/s)]

%% === 仿真 ===
param.ctrl_dt = 0.01;
end

function fp = zx_fit_flat_plate_params(param)
% Quan et al. model family, scaled from the ZX CFD table instead of copied.
alpha = param.bigalpha_arr(:);
CL = param.aero.CL_arr(:);
CD = param.aero.CD_arr(:);

fp.c0 = interp1(alpha, CD, 0, 'linear', 'extrap');

idx_slope = abs(alpha) <= 6 * param.D2R;
p = polyfit(alpha(idx_slope), CL(idx_slope), 1);
fp.c2 = max(abs(p(1)), 0.1);
fp.c3 = max(0.25 * fp.c2, 0.1);

alpha_hi = alpha(end);
sin2_hi = sin(2 * alpha_hi);
if abs(sin2_hi) > 1e-6
    fp.c1 = max(abs(CL(end) / sin2_hi), 0.1);
else
    fp.c1 = 0.9;
end

fp.alpha0 = 3 * param.D2R;
fp.kL = 38;
fp.kD = 48;
fp.trusted_min = alpha(1);
fp.trusted_max = alpha(end);
end

function [CD_full, CL_full] = zx_build_full_aero_tables(param)
alpha_full = param.aero_fullalpha_arr(:);
CL_full = zeros(size(alpha_full));
CD_full = zeros(size(alpha_full));

alpha_cfd = param.bigalpha_arr(:);
CL_cfd = param.aero.CL_arr(:);
CD_cfd = param.aero.CD_arr(:);
fp = param.aero.flat_plate;

CL_min = CL_cfd(1);
CD_min = CD_cfd(1);
CL_max = CL_cfd(end);
CD_max = CD_cfd(end);
blend = max(param.aero_extension_blend, 1e-6);

for i = 1:numel(alpha_full)
    a = alpha_full(i);
    if a >= fp.trusted_min && a <= fp.trusted_max
        CL_full(i) = interp1(alpha_cfd, CL_cfd, a, 'linear');
        CD_full(i) = interp1(alpha_cfd, CD_cfd, a, 'linear');
        continue;
    end

    [CD_emp, CL_emp] = zx_flat_plate_coeff(a, fp);
    if a > fp.trusted_max
        [CD_edge_emp, CL_edge_emp] = zx_flat_plate_coeff(fp.trusted_max, fp);
        if abs(CL_edge_emp) > 1e-9
            CL_emp = CL_emp * CL_max / CL_edge_emp;
        end
        if abs(CD_edge_emp - fp.c0) > 1e-9
            CD_emp = fp.c0 + (CD_emp - fp.c0) * (CD_max - fp.c0) / (CD_edge_emp - fp.c0);
        end
        w = zx_smoothstep(min((a - fp.trusted_max) / blend, 1));
        CL_full(i) = (1 - w) * CL_max + w * CL_emp;
        CD_full(i) = (1 - w) * CD_max + w * CD_emp;
    else
        w = zx_smoothstep(min((fp.trusted_min - a) / blend, 1));
        CL_full(i) = (1 - w) * CL_min + w * CL_emp;
        CD_full(i) = (1 - w) * CD_min + w * CD_emp;
    end
    CD_full(i) = max(CD_full(i), 0.001);
end

CD_full = CD_full(:).';
CL_full = CL_full(:).';
end

function [CD, CL] = zx_flat_plate_coeff(alpha, fp)
denom = (fp.c2 - fp.c3) * cos(alpha)^2 + fp.c3;
CL_s = 0.5 * fp.c2^2 / denom * sin(2 * alpha);
CD_s = fp.c0 + (fp.c2 * fp.c3 / denom) * sin(alpha)^2;
CL_l = fp.c1 * sin(2 * alpha);
CD_l = fp.c0 + 2 * fp.c1 * sin(alpha)^2;

sigma_L = zx_sigma(fp.alpha0, fp.kL, alpha);
sigma_D = zx_sigma(fp.alpha0, fp.kD, alpha);
CL = CL_s * sigma_L + CL_l * (1 - sigma_L);
CD = CD_s * sigma_D + CD_l * (1 - sigma_D);
end

function sigma = zx_sigma(alpha0, k, alpha)
sigma = (1 + tanh(k * alpha0^2 - k * alpha^2)) / (1 + tanh(k * alpha0^2));
end

function y = zx_smoothstep(x)
x = min(max(x, 0), 1);
y = x * x * (3 - 2 * x);
end
