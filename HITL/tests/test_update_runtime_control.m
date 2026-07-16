function test_update_runtime_control()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

clear update_runtime_control
cfg = hitl_config();
cfg.runtime_control.enable_file_control = true;
cfg.runtime_control.check_period = 0;
cfg.runtime_control.file = fullfile(tempdir, "hitl_runtime_control_test.txt");

cleanup = onCleanup(@() cleanup_file(cfg.runtime_control.file));

write_control_file(cfg.runtime_control.file, "force_enable=0");
cfg.model.force_enable = 1;
cfg = update_runtime_control(cfg, 0);
assert(cfg.model.force_enable == 0, "force_enable=0 should disable model forces.");

write_control_file(cfg.runtime_control.file, "force_enable=1");
cfg = update_runtime_control(cfg, 0.1);
assert(cfg.model.force_enable == 1, "force_enable=1 should enable model forces.");

write_control_file(cfg.runtime_control.file, "force_enable=bad");
cfg = update_runtime_control(cfg, 0.2);
assert(cfg.model.force_enable == 1, "Bad runtime_control format should keep the current value.");
end

function write_control_file(file, text)
fid = fopen(file, "w");
assert(fid > 0, "Could not open temporary runtime control file.");
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", text);
end

function cleanup_file(file)
if isfile(file)
    delete(file);
end
end
