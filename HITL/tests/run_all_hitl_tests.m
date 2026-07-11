function run_all_hitl_tests()
this_dir = fileparts(mfilename("fullpath"));
root = fileparts(this_dir);
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

tests = {@test_actuator_from_servo_output_raw, @test_user_hitl_config, @test_apply_user_initial_conditions, @test_stand_takeoff_state_step, @test_advance_hitl_dynamics_substeps, @test_prepare_stand_static_for_hitl, @test_initial_geodetic_position, @test_state_to_uavdata_like, ...
    @test_hil_state_quaternion_payload, @test_mavlink_encode_hil_state_quaternion, @test_integrate_aircraft_step, @test_openloop_model};
for k = 1:numel(tests)
    fprintf("Running %s...\n", func2str(tests{k}));
    tests{k}();
end
fprintf("All HITL tests passed.\n");
end




