function payload = uavdata_to_hil_state_quaternion_payload(uav, cfg)
%UAVDATA_TO_HIL_STATE_QUATERNION_PAYLOAD Convert UAV-like struct to MAVLink payload.

q = quat_normalize(uav.q_eb(:));

payload = struct();
payload.time_usec = uint64(uav.time_s * 1e6);
payload.attitude_quaternion = single(q(:).');
payload.rollspeed = single(uav.pqr(1));
payload.pitchspeed = single(uav.pqr(2));
payload.yawspeed = single(uav.pqr(3));
payload.lat = int32(uav.lat_deg * 1e7);
payload.lon = int32(uav.lon_deg * 1e7);
payload.alt = int32(uav.AMSL * 1000);
payload.vx = int16(uav.Ve(1) * 100);
payload.vy = int16(uav.Ve(2) * 100);
payload.vz = int16(uav.Ve(3) * 100);
payload.ind_airspeed = uint16(max(uav.EAS, 0) * 100);
payload.true_airspeed = uint16(max(uav.TAS, 0) * 100);
payload.xacc = int16(uav.ab(1) / cfg.env.g * 1000);
payload.yacc = int16(uav.ab(2) / cfg.env.g * 1000);
payload.zacc = int16(uav.ab(3) / cfg.env.g * 1000);
end
