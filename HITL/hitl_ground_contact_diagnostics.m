function diag = hitl_ground_contact_diagnostics(x, param)
%HITL_GROUND_CONTACT_DIAGNOSTICS Read permanent ground-contact activity.

x = x(:);
q_eb = quat_normalize(x(7:10));
R_eb = quat_to_dcm_be(q_eb).';

diag = struct();
diag.active = false(1, 0);
diag.active_contact_count = 0;
diag.contact_count_total = 0;
diag.info = struct();

if ~isfield(param, "ground") || ~isfield(param.ground, "contact_points_b")
    return;
end

[~, ~, info] = zx_ground_contact_force(x(1:3), x(4:6), R_eb, x(11:13), param);
diag.info = info;
diag.active = logical(info.active);
diag.active_contact_count = sum(diag.active);
diag.contact_count_total = numel(diag.active);
end
