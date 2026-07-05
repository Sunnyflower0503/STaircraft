function [Thrust, M_prop, n_rps, CT] = tandem_rotor_thrust(delta_t, alpha, beta, TAS, prop_spin, param, prop_angle)
%TANDEM_ROTOR_THRUST  77RC 2306 + 8x6 prop steady power-balance model.
%
% The previous model used a direct throttle-to-RPM polynomial.  This version
% solves RPM from P_motor(delta_t)=P_shaft with the measured axial inflow.

    if nargin < 7
        prop_angle = param.prop_angle(1);
    end

    D = 0.2032;
    D4 = D^4;
    D5 = D^5;
    rho = get_air_density(param);

    delta_t = min(max(delta_t, 0), 1);

    ca = cos(prop_angle);
    sa = sin(prop_angle);
    V_axial = TAS*cos(alpha)*cos(beta)*ca - TAS*sin(alpha)*cos(beta)*sa;

    % Coefficients fitted from ST建模.xlsx vehicle inflow data.
    % P_motor = p2*delta_t^2 + p1*delta_t + p0, W
    P_motor_coef = [16.501776, 160.89067, -54.531088];

    % C_X = a*J^2 + b*J + c
    CT_coef = [-0.44128219, 0.067079209, 0.095811585];
    CM_coef = [-0.033493272, 0.020157983, 0.0072201342];
    CP_coef = [-0.21044444, 0.12665634, 0.045365441];

    P_motor = max(polyval(P_motor_coef, delta_t), 0);
    n_rps = solve_rps_from_power(P_motor, V_axial, rho, D, CP_coef);

    if n_rps <= 0
        Thrust = 0;
        M_prop = 0;
        CT = 0;
        return;
    end

    J = V_axial/(n_rps*D);
    CT = polyval(CT_coef, J);
    CM = polyval(CM_coef, J);

    if CT <= 0
        Thrust = 0;
        M_prop = 0;
        return;
    end

    CM = max(CM, 0);
    Thrust = CT*rho*n_rps^2*D4;
    M_prop = prop_spin*CM*rho*n_rps^2*D5;
end

function rho = get_air_density(param)
    rho = 1.225;
    if isstruct(param)
        if isfield(param, 'rho') && isfinite(param.rho) && param.rho > 0
            rho = param.rho;
        elseif isfield(param, 'air_density') && isfinite(param.air_density) && param.air_density > 0
            rho = param.air_density;
        end
    end
end

function n_rps = solve_rps_from_power(P_motor, V0, rho, D, CP_coef)
    if P_motor <= 0
        n_rps = 0;
        return;
    end

    aP = CP_coef(1);
    bP = CP_coef(2);
    cP = CP_coef(3);

    f = @(n) rho*(cP*D^5*n.^3 + bP*D^4*V0*n.^2 + aP*D^3*V0^2*n) - P_motor;

    lo = 0;
    hi = 10;
    while f(hi) < 0 && hi < 20000
        hi = 2*hi;
    end

    if f(hi) < 0
        n_rps = 0;
        return;
    end

    for k = 1:80
        mid = 0.5*(lo + hi);
        if f(mid) >= 0
            hi = mid;
        else
            lo = mid;
        end
    end

    n_rps = 0.5*(lo + hi);
end
