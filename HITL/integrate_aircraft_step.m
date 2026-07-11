function x_next = integrate_aircraft_step(t, x, u, param, cfg)
%INTEGRATE_AIRCRAFT_STEP Advance the plant state according to force_enable.

if cfg.model.force_enable == 1
    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t t + cfg.dt], x);
    x_next = z(:, end);
else
    switch string(cfg.model.zero_force_mode)
        case "freeze"
            x_next = x;
        case "zero_force"
            x_next = zero_force_step(x, cfg.dt);
        otherwise
            warning("integrate_aircraft_step:UnknownZeroForceMode", ...
                "Unknown zero_force_mode '%s'; freezing state.", cfg.model.zero_force_mode);
            x_next = x;
    end
end

x_next(7:10) = quat_normalize(x_next(7:10));
end

function x_next = zero_force_step(x, dt)
x_next = x(:);
omega_b = x_next(11:13);
q = quat_normalize(x_next(7:10));

x_next(1:3) = x_next(1:3) + x_next(4:6) * dt;
Omega = [0,          -omega_b(1), -omega_b(2), -omega_b(3);
         omega_b(1),  0,           omega_b(3), -omega_b(2);
         omega_b(2), -omega_b(3),  0,           omega_b(1);
         omega_b(3),  omega_b(2), -omega_b(1),  0];
x_next(7:10) = q + 0.5 * Omega * q * dt;
end
