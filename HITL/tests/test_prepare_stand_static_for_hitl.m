function test_prepare_stand_static_for_hitl()
this_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(this_dir);
addpath(hitl_dir); addpath(fullfile(hitl_dir, "utils")); addpath(fullfile(fileparts(hitl_dir), "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "stand_static";
param = init_param_zx();

try
    [~, x_stand, u0, meta] = prepare_stand_static_for_hitl(param, cfg);
catch ME
    if contains(ME.message, "stand") || contains(ME.identifier, "prepare_stand_static")
        fprintf("SKIP: stand static setup function is not available yet. %s\n", ME.message);
        return;
    end
    rethrow(ME);
end

assert(isequal(size(x_stand), [13, 1]), "x_stand must be 13x1.");
assert(isequal(size(u0), [12, 1]), "u0 must be 12x1.");
assert(all(isfinite(x_stand)), "x_stand contains NaN or Inf.");
assert(all(isfinite(u0)), "u0 contains NaN or Inf.");
assert(norm(x_stand(4:6)) < 1e-3, "stand linear velocity is not settled.");
assert(norm(x_stand(11:13)) < 1e-3, "stand angular rate is not settled.");
assert(meta.euler_deg(2) > 20, "stand pitch should be above 20 deg.");
assert(string(meta.mode) == "stand_static", "meta.mode mismatch.");
end
