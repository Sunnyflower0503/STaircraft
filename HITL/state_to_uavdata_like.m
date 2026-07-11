function uav = state_to_uavdata_like(t, x, u, param, cfg)
%STATE_TO_UAVDATA_LIKE Convert model state to the subset used by HITL TX.

x = x(:);
p_e = x(1:3);
v_e = x(4:6);
q_eb = quat_normalize(x(7:10));
omega_b = x(11:13);

[lat0, lon0, amsl0] = initial_geo(param, cfg);
earth_radius = cfg.env.earth_radius;
lat = lat0 + p_e(1) / earth_radius * 180 / pi;
cos_lat0 = max(cosd(lat0), 1e-8);
lon = lon0 + p_e(2) / (earth_radius * cos_lat0) * 180 / pi;
amsl = amsl0 - p_e(3);

wind_e = zeros(3, 1);
if isfield(param, "wind")
    wind_e = param.wind(:);
end
rho = cfg.env.rho0;
if isfield(param, "rho")
    rho = param.rho;
end
TAS = norm(v_e - wind_e);
EAS = TAS * sqrt(max(rho, 0) / cfg.env.rho0);

% TODO: confirm whether ab matches the original Simulink specific-force convention exactly.
ab = estimate_body_accel(t, x, u, param, cfg, q_eb);

uav = struct();
uav.time_s = t;
uav.Xe = p_e;
uav.Ve = v_e;
uav.q_eb = q_eb;
uav.DCM_be = quat_to_dcm_be(q_eb);
uav.pqr = omega_b;
uav.lat_deg = lat;
uav.lon_deg = lon;
uav.AMSL = amsl;
uav.ab = ab;
uav.TAS = TAS;
uav.EAS = EAS;
end

function [lat0, lon0, amsl0] = initial_geo(param, cfg)
lat0 = cfg.init.lat_deg;
lon0 = cfg.init.lon_deg;
amsl0 = cfg.init.AMSL;

if isfield(cfg.init, "use_param_geodetic") && cfg.init.use_param_geodetic && isfield(param, "InitData")
    init = param.InitData;
    if isfield(init, "lat_deg"), lat0 = init.lat_deg; end
    if isfield(init, "lon_deg"), lon0 = init.lon_deg; end
    if isfield(init, "AMSL"), amsl0 = init.AMSL; end
end
end

function ab = estimate_body_accel(t, x, u, param, cfg, q_eb)
try
    dx = tandem_zx_dynamics(t, x, u, param);
    accel_e = dx(4:6);
    dcm_be = quat_to_dcm_be(q_eb);
    gravity_e = [0; 0; cfg.env.g];
    ab = dcm_be * (accel_e - gravity_e);
catch
    ab = zeros(3, 1);
end
end


