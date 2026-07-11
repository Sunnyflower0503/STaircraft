function cfg = update_runtime_control(cfg, t)
%UPDATE_RUNTIME_CONTROL Poll runtime_control.txt for force_enable.

persistent last_check_t last_force_enable

if ~isfield(cfg, "runtime_control") || ~cfg.runtime_control.enable_file_control
    return;
end

if isempty(last_check_t)
    last_check_t = -inf;
end
if isempty(last_force_enable)
    last_force_enable = cfg.model.force_enable;
end

if t - last_check_t < cfg.runtime_control.check_period
    return;
end
last_check_t = t;

control_file = cfg.runtime_control.file;
if ~isfile(control_file)
    return;
end

try
    text = fileread(control_file);
    token = regexp(text, "force_enable\s*=\s*([01])", "tokens", "once");
    if isempty(token)
        warning("update_runtime_control:BadFormat", ...
            "Ignoring runtime control file with no force_enable entry: %s", control_file);
        return;
    end

    new_force_enable = str2double(token{1});
    if new_force_enable ~= cfg.model.force_enable
        old_force_enable = cfg.model.force_enable;
        cfg.model.force_enable = new_force_enable;
        fprintf("[HITL runtime] t=%.2f force_enable: %d -> %d\n", ...
            t, old_force_enable, new_force_enable);
    end
    last_force_enable = cfg.model.force_enable;
catch ME
    warning("update_runtime_control:ReadFailed", ...
        "Could not read runtime control file '%s': %s", control_file, ME.message);
    cfg.model.force_enable = last_force_enable;
end
end
