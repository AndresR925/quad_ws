function config = defaultServoCommandConfig()
%DEFAULTSERVOCOMMANDCONFIG Default calibration for 8 position servos.
%   Edit these values according to the zero, direction, and range of each
%   servo in the real robot.

template = struct( ...
    'invert', false, ...
    'scale', 1.0, ...
    'offset_deg', 90.0, ...
    'min_deg', 0.0, ...
    'max_deg', 180.0);

config.hip_fl = template;
config.knee_fl = template;
config.hip_fr = template;
config.knee_fr = template;
config.hip_rl = template;
config.knee_rl = template;
config.hip_rr = template;
config.knee_rr = template;

% Optional trajectory trimming and resampling.
config.sample_time = [];
config.start_time = [];
config.end_time = [];
end
