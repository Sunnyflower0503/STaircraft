function payload = uavdata_to_hil_sensor_payload(uav, cfg)
%UAVDATA_TO_HIL_SENSOR_PAYLOAD Build coherent simulated IMU/environment data.

specific_force_b = double(uav.ab(:));
omega_b = double(uav.pqr(:));
if numel(specific_force_b) ~= 3 || numel(omega_b) ~= 3 || ...
        any(~isfinite([specific_force_b; omega_b]))
    error("uavdata_to_hil_sensor_payload:BadKinematics", ...
        "Body specific force and angular rate must be finite 3-vectors.");
end

% Representative NED geomagnetic field in gauss, rotated into body axes.
mag_ned_gauss = [0.21523; 0.0; 0.42741];
mag_b_gauss = double(uav.DCM_be) * mag_ned_gauss;

altitude_m = double(uav.AMSL);
pressure_ratio_base = max(1.0 - 2.25577e-5 * altitude_m, 0.05);
abs_pressure_hpa = 1013.25 * pressure_ratio_base ^ 5.25588;
rho = double(cfg.env.rho0);
diff_pressure_hpa = 0.5 * rho * max(double(uav.TAS), 0) ^ 2 / 100.0;

payload = struct();
payload.time_usec = uint64(max(double(uav.time_s), 0) * 1e6);
payload.xacc = single(specific_force_b(1));
payload.yacc = single(specific_force_b(2));
payload.zacc = single(specific_force_b(3));
payload.xgyro = single(omega_b(1));
payload.ygyro = single(omega_b(2));
payload.zgyro = single(omega_b(3));
payload.xmag = single(mag_b_gauss(1));
payload.ymag = single(mag_b_gauss(2));
payload.zmag = single(mag_b_gauss(3));
payload.abs_pressure = single(abs_pressure_hpa);
payload.diff_pressure = single(diff_pressure_hpa);
payload.pressure_alt = single(altitude_m);
payload.temperature = single(15.0 - 0.0065 * altitude_m);
payload.fields_updated = uint32(hex2dec("1FFF"));
payload.id = uint8(0);
end
