function [param, x_hover, u0, meta] = prepare_airborne_nose_up_hover_for_hitl(param, cfg)
%PREPARE_AIRBORNE_NOSE_UP_HOVER_FOR_HITL Prepare a nose-up airborne hover pose.

hitl_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(fileparts(hitl_dir), "matlab_model"));

param.ground.enable = false;
u0 = zeros(12, 1);

if isfield(cfg, "hover") && isfield(cfg.hover, "u0")
    candidate_u0 = double(cfg.hover.u0(:));
    if numel(candidate_u0) == 12 && all(isfinite(candidate_u0))
        u0 = candidate_u0;
    end
end

altitude_m = 20;
if isfield(cfg, "hover") && isfield(cfg.hover, "altitude_m")
    altitude_m = double(cfg.hover.altitude_m);
end

euler_deg = [0; 90; cfg.init.heading_deg];
if isfield(cfg, "hover") && isfield(cfg.hover, "Euler_deg")
    euler_deg = double(cfg.hover.Euler_deg(:));
end

q0 = euler_to_quat_wxyz(euler_deg(1) * param.D2R, euler_deg(2) * param.D2R, euler_deg(3) * param.D2R);
p0 = [0; 0; -abs(altitude_m)];
x_hover = [p0; zeros(3, 1); q0(:); zeros(3, 1)];

meta = struct();
meta.mode = "airborne_nose_up_hover";
meta.cache_used = false;
meta.cache_file = "";
meta.position_ned = x_hover(1:3);
meta.q_eb = x_hover(7:10);
meta.euler_deg = euler_deg;
meta.velocity_norm = 0;
meta.angular_rate_norm = 0;
meta.altitude_m = abs(altitude_m);
meta.ground_enable = param.ground.enable;

if any(~isfinite(x_hover)) || any(~isfinite(u0))
    error("prepare_airborne_nose_up_hover_for_hitl:NonFinite", ...
        "Airborne hover state or input contains NaN/Inf.");
end
end
