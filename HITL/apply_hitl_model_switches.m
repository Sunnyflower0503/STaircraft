function param = apply_hitl_model_switches(param, cfg)
%APPLY_HITL_MODEL_SWITCHES Apply user-editable HITL model switches to param.

if isfield(cfg, "model") && isfield(cfg.model, "slipstream_enable")
    param.slipstream_enable = logical(cfg.model.slipstream_enable);
end
if isfield(cfg, "model") && isfield(cfg.model, "slipstream_ff_enable")
    param.slipstream_ff_enable = logical(cfg.model.slipstream_ff_enable);
end
if isfield(cfg, "model") && isfield(cfg.model, "aero_body_enable")
    param.aero_body_enable = logical(cfg.model.aero_body_enable);
end
end
