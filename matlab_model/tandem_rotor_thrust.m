function [Thrust, M_prop, n_rps, CT] = tandem_rotor_thrust(delta_t, alpha, beta, TAS, prop_spin, param, prop_angle)
% ROTOR thrust/torque model (exact C code match)
    if nargin < 7
        prop_angle = param.prop_angle(1);
    end
    n_rpm = polyval([-3424 12810 -4], delta_t);
    n_rps = max(n_rpm/60, 0);
    if n_rps <= 0, Thrust=0; M_prop=0; CT=0; return; end
    ca = cos(prop_angle); sa = sin(prop_angle);
    V_axial = TAS*cos(alpha)*cos(beta)*ca - TAS*sin(alpha)*cos(beta)*sa;
    J = V_axial/(n_rps*0.2032);
    if J > 0.8
        CT = 0; CM = 0;
    else
        CT = polyval([0.3101 -0.5816 0.1747 0.09582], J);
        CM = polyval([-0.0169 0.0071 0.0103], J);
    end
    if CT <= 0, Thrust=0; M_prop=0; return; end
    Thrust = n_rps^2*CT*0.0017048839192575996*1.225;
    M_prop = prop_spin*CM*n_rps^2*0.00034643241239314424*1.225;
end
