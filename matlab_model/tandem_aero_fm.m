function [f_a, m_a, slip_diag] = tandem_aero_fm(Ve, DCM_be, omega_b, delta_aeL, delta_aeR, delta_t, varargin)
%TANDEM_AERO_FM  Aerocraft/aero F&M in body axis.
%
% Reproduction of the Tandem_zx_trans6 Aerocraft aerodynamic blocks:
%   S_free static table, Sw dynamic derivatives, elevon increments, and
%   eight S_slip force panels.  CL/CD can use the Quan et al. full-angle
%   flat-plate/post-stall extension generated in init_param_zx.

if nargin == 7
    param = varargin{1};
    mode_out = get_optional_field(param, 'mode_out', []);
elseif nargin >= 8
    mode_out = varargin{1};
    param = varargin{2};
else
    error('tandem_aero_fm:BadInput', 'Missing param input.');
end

Ve = Ve(:);
omega_b = omega_b(:);
delta_t = delta_t(:); %#ok<NASGU>  % Interface kept aligned with Aerocraft.

DCM_eb = DCM_be';
Va_b = DCM_eb * (Ve - param.wind(:));
Va = norm(Va_b);

if Va < 1e-9
    alpha = 0;
    beta = 0;
else
    alpha = atan2(Va_b(3), Va_b(1));
    beta = atan2(Va_b(2), hypot(Va_b(1), Va_b(3)));
end
qbar = 0.5 * param.rho * Va^2;

S = param.S;
S_free = param.S_free;
b = param.b;
MAC = param.MAC;

alpha_dyn = min(max(alpha, -0.5), 0.5);
if Va < 1e-9
    p_hat = 0;
    q_hat = 0;
    r_hat = 0;
else
    p_hat = omega_b(1) * b / (2 * Va);
    q_hat = omega_b(2) * MAC / (2 * Va);
    r_hat = omega_b(3) * b / (2 * Va);
end
alpha_dot_hat = get_optional_field(param, 'alpha_dot_hat', 0);

K = aero_enable(mode_out);

%% Elevon force increments: S158/S159
CD_de_L = param.aero.CDdelta_aeL * delta_aeL;
CL_de_L = param.aero.CLdelta_aeL * delta_aeL;
CD_de_R = param.aero.CDdelta_aeR * delta_aeR;
CL_de_R = param.aero.CLdelta_aeR * delta_aeR;

F_de_L = qbar * S * aeroforce2bodyaxis(alpha, beta, CD_de_L, 0, CL_de_L);
F_de_R = qbar * S * aeroforce2bodyaxis(alpha, beta, CD_de_R, 0, CL_de_R);
F_elevon = F_de_L + F_de_R;

M_elevon = qbar * S * [ ...
    b   * (param.aero.Cldelta_aeL * delta_aeL + param.aero.Cldelta_aeR * delta_aeR); ...
    MAC * (param.aero.Cmdelta_aeL * delta_aeL + param.aero.Cmdelta_aeR * delta_aeR); ...
    b   * (param.aero.Cndelta_aeL * delta_aeL + param.aero.Cndelta_aeR * delta_aeR)];

%% S_free: S267/S269/S274
CD_static = aero_cd(param, alpha);
CL_static = aero_cl(param, alpha);
Cm_static = interp1_table(param.bigalpha_arr, param.aero.Cm_arr, alpha);

F_sfree = qbar * S_free * aeroforce2bodyaxis(alpha, beta, CD_static, 0, CL_static);
M_sfree = K * qbar * S_free * [0; MAC * Cm_static; 0];

%% Sw: S268/S283/S286-S289
CY_dyn = param.aero.CYbeta * beta ...
       + interp1_table(param.alpha_arr, param.aero.CYp_arr, alpha_dyn) * p_hat ...
       + interp1_table(param.alpha_arr, param.aero.CYr_arr, alpha_dyn) * r_hat;
CL_dyn = param.aero.CLq * q_hat + param.aero.CLalpdot * alpha_dot_hat;

F_sw = K * qbar * S * aeroforce2bodyaxis(alpha, beta, 0, CY_dyn, CL_dyn);

Cl_dyn = interp1_table(param.alpha_arr, param.aero.Clbeta_arr, alpha_dyn) * beta ...
       + interp1_table(param.alpha_arr, param.aero.Clp_arr, alpha_dyn) * p_hat ...
       + interp1_table(param.alpha_arr, param.aero.Clr_arr, alpha_dyn) * r_hat;
Cm_dyn = param.aero.Cmq * q_hat + param.aero.Cmalpdot * alpha_dot_hat;
Cn_dyn = interp1_table(param.alpha_arr, param.aero.Cnbeta_arr, alpha_dyn) * beta ...
       + interp1_table(param.alpha_arr, param.aero.Cnp_arr, alpha_dyn) * p_hat ...
       + interp1_table(param.alpha_arr, param.aero.Cnr_arr, alpha_dyn) * r_hat;

M_sw = K * qbar * S * [b * Cl_dyn; MAC * Cm_dyn; b * Cn_dyn];

%% S_slip force panels: S184-S191
F_slip = zeros(3, 1);
slipstream_enable = get_optional_field(param, 'slipstream_enable', true);
slip_diag = init_slip_diag();
for ip = 1:8
    if slipstream_enable
        dt_i = sat(delta_t(ip), get_optional_field(param, 'rotor_throttle_min', 0), 1);
        [T_i, ~, n_rps_i, CT_i] = tandem_rotor_thrust(dt_i, alpha, beta, Va, ...
                                                     param.prop_spin(ip), param, ...
                                                     param.prop_angle(ip));
        slip_state = paper_slipstream_state(T_i, Va_b, param);
    else
        dt_i = sat(delta_t(ip), get_optional_field(param, 'rotor_throttle_min', 0), 1);
        [T_i, ~, n_rps_i, CT_i] = tandem_rotor_thrust(dt_i, alpha, beta, Va, ...
                                                     param.prop_spin(ip), param, ...
                                                     param.prop_angle(ip));
        slip_state = paper_slipstream_state(0, Va_b, param);
    end

    alpha_slip = alpha;
    beta_slip = beta;
    Va_slip = slip_state.Ve_s;
    qbar_slip = qbar;

    if Va_slip >= 1e-9
        alpha_local = slip_state.alpha_s;
        beta_local = slip_state.beta_s;
    else
        alpha_local = alpha;
        beta_local = beta;
    end

    if get_optional_field(param, 'alpha_slip_switch', 1) > 0.5
        alpha_slip = alpha_local;
        beta_slip = beta_local;
    end

    if get_optional_field(param, 'DP_slip_switch', 1) > 0.5
        qbar_slip = 0.5 * param.rho * Va_slip^2;
    end

    if slipstream_enable && get_optional_field(param, 'slipstream_ff_enable', true)
        f_s = slipstream_lift_factor(slip_state.Ve_s, Va, alpha_slip, param);
    else
        f_s = 1;
    end

    S_slip_i = slipstream_panel_area(ip, param);
    CL_free = aero_cl(param, alpha_slip);
    CD_free = aero_cd(param, alpha_slip);
    CL_slip = f_s * CL_free;
    CD_slip = slipstream_drag_coefficient(CD_free, CL_free, f_s, param);
    CA_slip = aeroforce2bodyaxis(alpha_slip, beta_slip, CD_slip, 0, CL_slip);
    F_slip_i = qbar_slip * S_slip_i * CA_slip;
    F_slip = F_slip + F_slip_i;

    slip_diag.T(ip) = T_i;
    slip_diag.n_rps(ip) = n_rps_i;
    slip_diag.CT(ip) = CT_i;
    slip_diag.Vi0(ip) = slip_state.Vi0;
    slip_diag.Vi_eff(ip) = slip_state.Vi_eff;
    slip_diag.Rs(ip) = slip_state.Rs;
    slip_diag.f_s(ip) = f_s;
    slip_diag.S_slip_i(ip) = S_slip_i;
    slip_diag.alpha_s(ip) = alpha_slip;
    slip_diag.beta_s(ip) = beta_slip;
    slip_diag.qbar_s(ip) = qbar_slip;
    slip_diag.CL_free(ip) = CL_free;
    slip_diag.CL_slip(ip) = CL_slip;
    slip_diag.CD_slip(ip) = CD_slip;
    slip_diag.F_body(:, ip) = F_slip_i;
end

f_a = F_elevon + F_sfree + F_sw + F_slip;
m_a = M_elevon + M_sfree + M_sw;
end

function diag = init_slip_diag()
diag = struct();
fields = {'T', 'n_rps', 'CT', 'Vi0', 'Vi_eff', 'Rs', 'f_s', 'S_slip_i', ...
          'alpha_s', 'beta_s', 'qbar_s', 'CL_free', 'CL_slip', 'CD_slip'};
for i = 1:numel(fields)
    diag.(fields{i}) = zeros(8, 1);
end
diag.F_body = zeros(3, 8);
end

function state = paper_slipstream_state(T, Va_b, param)
D = get_optional_field(param, 'prop_D', 0.2032);
Rp = 0.5 * D;
A = pi * Rp^2;
rho = param.rho;
x = max(get_optional_field(param, 'slip_x', get_optional_field(param, 'MAC', D)), 0);

Vinf = max(Va_b(1), 0);
if T > 0 && rho > 0 && A > 0
    Vi0 = 0.5 * (-Vinf + sqrt(max(Vinf^2 + 2 * T / (rho * A), 0)));
else
    Vi0 = 0;
end

if Rp > 0
    x_over_Rp = x / Rp;
    Vi_eff = Vi0 * (1 + x_over_Rp / sqrt(1 + x_over_Rp^2));
else
    Vi_eff = Vi0;
end

if Vi_eff > 1e-12
    Rs = Rp * sqrt(max(Vi0 / Vi_eff, 0));
else
    Rs = Rp;
end

u_s = Va_b(1) + Vi_eff;
v_s = Va_b(2);
w_s = Va_b(3);
Ve_s = sqrt(u_s^2 + v_s^2 + w_s^2);

if Ve_s >= 1e-12
    alpha_s = atan2(w_s, u_s);
    beta_s = atan2(v_s, hypot(u_s, w_s));
else
    alpha_s = 0;
    beta_s = 0;
end

state = struct('Vi0', Vi0, 'Vi_eff', Vi_eff, 'Rs', Rs, 'Ve_s', Ve_s, ...
               'alpha_s', alpha_s, 'beta_s', beta_s);
end

function f_s = slipstream_lift_factor(Ve_s, Vinf, alpha_s, param)
den = Ve_s^2 + Vinf^2;
if den <= 1e-12
    f_s = 1;
    return;
end

a0 = get_optional_field(param, 'slip_a0', 0.3 / (0.5 * 0.2032));
a1 = get_optional_field(param, 'slip_a1', 0.5);
lambda = (Ve_s^2 - Vinf^2) / den;
lambda = min(max(lambda, -0.999999), 0.999999);
sa = sin(alpha_s);

d1 = protect_denominator(1 + 4 * a1^2 / a0^2 + 4 * a1 / a0 * sa);
d2 = protect_denominator(1 + 4 * a1^2 / a0^2 - 4 * a1 / a0 * sa);
d3 = protect_denominator(1 + 4 / a0^2 + 4 / a0 * sa);
d4 = protect_denominator(1 + 4 / a0^2 - 4 / a0 * sa);

sigma = 1 + lambda / d1 + lambda / d2 + lambda^2 / d3 + lambda^2 / d4;
if ~isfinite(sigma) || abs(sigma) <= 1e-9
    f_s = 1;
else
    f_s = 1 / sigma;
end
end

function value = protect_denominator(value)
if abs(value) < 1e-9
    value = sign(value + (value == 0)) * 1e-9;
end
end

function S_slip_i = slipstream_panel_area(ip, param)
if get_optional_field(param, 'slip_area_from_paper', false)
    D = get_optional_field(param, 'prop_D', 0.2032);
    Rp = 0.5 * D;
    c = get_optional_field(param, 'slip_chord', get_optional_field(param, 'MAC', 0.3));
    S_slip_i = (2 - 1 / sqrt(2)) * Rp^2 + (1 / sqrt(2)) * Rp * c;
else
    S_slip_i = param.S_slip(ip);
end
end

function CD_slip = slipstream_drag_coefficient(CD_free, CL_free, f_s, param)
if isfield(param, 'aero') && isfield(param.aero, 'CD0') && isfield(param, 'slip_drag_k')
    CD_slip = param.aero.CD0 + f_s * param.slip_drag_k * CL_free^2;
else
    CD_slip = CD_free;
end
end

function K = aero_enable(mode_out)
if isempty(mode_out)
    K = 1;
elseif mode_out == 2
    K = 0;
else
    K = double(mode_out == 0 || mode_out == 1 || mode_out == 3);
end
end

function CL = aero_cl(param, alpha)
if isfield(param, 'aero_extend_mode') && strcmpi(param.aero_extend_mode, 'flat_plate') ...
        && isfield(param, 'aero_fullalpha_arr') && isfield(param.aero, 'CL_full_arr')
    CL = interp1_table(param.aero_fullalpha_arr, param.aero.CL_full_arr, wrap_to_pi_local(alpha));
else
    CL = interp1_table(param.bigalpha_arr, param.aero.CL_arr, alpha);
end
end

function CD = aero_cd(param, alpha)
if isfield(param, 'aero_extend_mode') && strcmpi(param.aero_extend_mode, 'flat_plate') ...
        && isfield(param, 'aero_fullalpha_arr') && isfield(param.aero, 'CD_full_arr')
    CD = interp1_table(param.aero_fullalpha_arr, param.aero.CD_full_arr, wrap_to_pi_local(alpha));
else
    CD = param.aero.CD0 + interp1_table(param.bigalpha_arr, param.aero.CD_arr, alpha);
end
end

function alpha = wrap_to_pi_local(alpha)
alpha = mod(alpha + pi, 2 * pi) - pi;
end

function value = get_optional_field(s, name, default_value)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function y = interp1_table(bp, tab, x)
bp = bp(:);
tab = tab(:);
if x <= bp(1)
    y = tab(1);
elseif x >= bp(end)
    y = tab(end);
else
    idx = find(bp <= x, 1, 'last');
    if idx >= numel(bp)
        y = tab(end);
    else
        frac = (x - bp(idx)) / (bp(idx + 1) - bp(idx));
        y = tab(idx) + frac * (tab(idx + 1) - tab(idx));
    end
end
end

function CA = aeroforce2bodyaxis(alpha, beta, CD, CY, CL)
CA = [ ...
    -CD * cos(alpha) * cos(beta) - CY * cos(alpha) * sin(beta) + CL * sin(alpha); ...
    -CD * sin(beta) + CY * cos(beta); ...
    -CD * sin(alpha) * cos(beta) - CY * sin(alpha) * sin(beta) - CL * cos(alpha)];
end
