function bridge = mavlink_backend_init(cfg)
%MAVLINK_BACKEND_INIT Initialize the configured MAVLink backend.

switch string(cfg.mavlink.backend)
    case "pymavlink"
        this_dir = fileparts(mfilename("fullpath"));
        if count(py.sys.path, this_dir) == 0
            insert(py.sys.path, int32(0), this_dir);
        end
        try
            mod = py.importlib.import_module("pymavlink_bridge");
            bridge = mod.MavlinkBridge(cfg.mavlink.sysid, cfg.mavlink.compid);
        catch ME
            error("mavlink_backend_init:PymavlinkUnavailable", ...
                "Failed to initialize pymavlink backend. Install with 'pip install pymavlink' in MATLAB's Python environment. Details: %s", ME.message);
        end
    otherwise
        error("mavlink_backend_init:UnsupportedBackend", ...
            "Unsupported MAVLink backend '%s'.", cfg.mavlink.backend);
end
end
