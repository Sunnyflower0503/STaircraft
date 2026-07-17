%TEST_HITL_LOG_WRITE_SMOKE Verify HITL/logs can be written without serial.

clear; clc;

test_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(test_dir);
logs_dir = fullfile(hitl_dir, "logs");
if ~exist(logs_dir, "dir")
    mkdir(logs_dir);
end

stats = struct();
stats.mode = "log_write_smoke";
stats.created_at = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
stats.history = struct("wall_time_s", 0, "plant_time_s", 0);

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
log_file = fullfile(logs_dir, "log_write_smoke_" + timestamp + ".mat");
save(log_file, "stats");

assert(isfile(log_file), "Log smoke file was not created.");
fprintf("HITL log write smoke PASS: %s\n", log_file);
