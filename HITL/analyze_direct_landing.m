function metrics = analyze_direct_landing(mat_file, plot_file, csv_file, metrics_file)
%ANALYZE_DIRECT_LANDING Export direct-landing model history, metrics, and plot.

loaded = load(mat_file, "result");
result = loaded.result;
h = result.history;
t = double(h.wall_time_s(:));
plant_t = double(h.plant_time_s(:));
pos = double(h.position_ned_m.');
vel = double(h.velocity_ned_mps.');
rpy = double(h.euler_deg.');
servo = double(h.servo_raw.');
mask = uint8(h.contact_mask(:));
force_enable = logical(h.force_enable(:));
contact_count = zeros(size(mask));
for bit_index = 1:6
    contact_count = contact_count + double(bitget(mask, bit_index));
end

release_index = find(force_enable, 1, "first");
rear_index = find(bitand(mask, uint8(56)) == uint8(56), 1, "first");
all_index = find(mask == uint8(63), 1, "first");
tip_index = find((1:numel(t)).' >= rear_index & servo(:, 7) >= 1880 & servo(:, 8) >= 1880, 1, "first");

assert(~isempty(release_index), "No dynamics-release sample in MAT history.");
assert(~isempty(rear_index), "No rear 3/3 contact in MAT history.");
assert(~isempty(all_index), "No all 6/6 contact in MAT history.");
assert(~isempty(tip_index), "No post-rear 1900 PWM wingtip sample in MAT history.");

plot_indices = release_index:numel(t);
t_release = t - t(release_index);

metrics = struct();
metrics.passed = logical(result.all_contacts_confirmed && ~result.front_before_rear && ~result.aborted);
metrics.front_before_rear = logical(result.front_before_rear);
metrics.release_wall_s = t(release_index);
metrics.rear_contact_wall_s = t(rear_index);
metrics.all_contact_wall_s = t(all_index);
metrics.landing_confirmed_wall_s = double(result.all_contacts_wall_s);
metrics.release_to_rear_s = t(rear_index) - t(release_index);
metrics.rear_to_all_s = t(all_index) - t(rear_index);
metrics.tip_90_wall_s = t(tip_index);
metrics.tip_90_delay_s = t(tip_index) - t(rear_index);
metrics.rear_down_speed_mps = vel(rear_index, 3);
metrics.rear_horizontal_speed_mps = hypot(vel(rear_index, 1), vel(rear_index, 2));
metrics.rear_pitch_deg = rpy(rear_index, 2);
metrics.all_pitch_deg = rpy(all_index, 2);
metrics.max_down_speed_mps = max(vel(release_index:all_index, 3));
metrics.max_horizontal_speed_mps = max(hypot(vel(release_index:all_index, 1), vel(release_index:all_index, 2)));
metrics.horizontal_displacement_to_rear_m = hypot(pos(rear_index, 1) - pos(release_index, 1), ...
    pos(rear_index, 2) - pos(release_index, 2));
metrics.horizontal_displacement_to_all_m = hypot(pos(all_index, 1) - pos(release_index, 1), ...
    pos(all_index, 2) - pos(release_index, 2));
metrics.tip_left_pwm_after_latch = median(servo(tip_index:all_index, 7), "omitnan");
metrics.tip_right_pwm_after_latch = median(servo(tip_index:all_index, 8), "omitnan");
metrics.final_contact_mask = double(mask(end));
metrics.final_altitude_agl_m = -pos(end, 3);

table_out = table(t, plant_t, pos(:, 1), pos(:, 2), -pos(:, 3), ...
    vel(:, 1), vel(:, 2), vel(:, 3), rpy(:, 1), rpy(:, 2), rpy(:, 3), ...
    double(mask), contact_count, servo(:, 7), servo(:, 8), double(force_enable), ...
    'VariableNames', {'wall_time_s', 'plant_time_s', 'north_m', 'east_m', 'altitude_agl_m', ...
    'vn_mps', 've_mps', 'vd_mps', 'roll_deg', 'pitch_deg', 'yaw_deg', ...
    'contact_mask', 'contact_count', 'main7_pwm', 'main8_pwm', 'force_enable'});
writetable(table_out, csv_file);

f = figure("Visible", "off", "Color", "w", "Position", [100 100 1400 900]);
layout = tiledlayout(f, 2, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Direct six-contact landing HITL replay");

nexttile;
plot(pos(release_index:all_index, 2), pos(release_index:all_index, 1), "b-", "LineWidth", 1.5); hold on;
plot(pos(release_index, 2), pos(release_index, 1), "go", "MarkerFaceColor", "g");
plot(pos(rear_index, 2), pos(rear_index, 1), "mo", "MarkerFaceColor", "m");
plot(pos(all_index, 2), pos(all_index, 1), "ks", "MarkerFaceColor", "k");
axis equal; grid on; xlabel("East (m)"); ylabel("North (m)");
legend("track", "release", "rear 3/3", "all 6/6", "Location", "best");
title("Ground track (no waypoint route)");

nexttile;
yyaxis left; plot(t_release(plot_indices), -pos(plot_indices, 3), "b-", "LineWidth", 1.3); ylabel("Altitude AGL (m)");
yyaxis right; plot(t_release(plot_indices), vel(plot_indices, 3), "r-", "LineWidth", 1.0); ylabel("Down speed (m/s)");
xline(t_release(rear_index), "m--", "rear 3/3"); xline(t_release(all_index), "k--", "all 6/6");
grid on; xlabel("Time since release (s)"); title("Vertical trajectory");

nexttile;
plot(t_release(plot_indices), rpy(plot_indices, 1), "LineWidth", 1.0); hold on;
plot(t_release(plot_indices), rpy(plot_indices, 2), "LineWidth", 1.3);
plot(t_release(plot_indices), rpy(plot_indices, 3), "LineWidth", 1.0);
xline(t_release(rear_index), "m--"); xline(t_release(all_index), "k--");
grid on; xlabel("Time since release (s)"); ylabel("Angle (deg)");
legend("roll", "pitch", "yaw", "Location", "best"); title("Attitude");

nexttile;
yyaxis left; stairs(t_release(plot_indices), contact_count(plot_indices), "k-", "LineWidth", 1.3); ylim([-0.2 6.5]); ylabel("Active contacts");
yyaxis right; plot(t_release(plot_indices), servo(plot_indices, 7), "b-", ...
    t_release(plot_indices), servo(plot_indices, 8), "r-", "LineWidth", 1.0); hold on;
yline(1900, "k:", "90% = 1900 PWM"); ylabel("Wingtip PWM");
xline(t_release(rear_index), "m--"); xline(t_release(all_index), "k--");
grid on; xlabel("Time since release (s)"); legend("MAIN7", "MAIN8", "Location", "best");
title("Contact sequence and independent wingtip command");

exportgraphics(f, plot_file, "Resolution", 180);
close(f);

fid = fopen(metrics_file, "w");
assert(fid >= 0, "Could not open metrics file for writing.");
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, jsonencode(metrics, "PrettyPrint", true), "char");
end
