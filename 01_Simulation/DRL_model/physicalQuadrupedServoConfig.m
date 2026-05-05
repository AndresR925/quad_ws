function config = physicalQuadrupedServoConfig()
%PHYSICALQUADRUPEDSERVOCONFIG Mapping from MATLAB gait to the physical robot.
%   This configuration maps the 8 joint trajectories of the 2-DOF
%   quadruped model to the muslo/canilla servos that are active in the
%   physical robot. The lateral cadera servos remain fixed at their rest
%   positions for this first real-robot integration step.

% Full PCA9685 layout from the user's Arduino sketch.
config.rest_deg = [ ...
    100, 90, 120, 90,...
   89, 120, 120, 90,...
   90, 98, 60, 60,...
   93, 90, 60, 90];


config.dir = [ ...
    -1,  -1, 1, 1, ...
     1,  1,  1, 1, ...
     1,  1, 1, -1, ...
     -1, -1,  1, 1];

config.safe_min_deg = [ ...
    85, 50, 40, 0,...
  74, 50, 40, 0,...
   0, 83, 20, 43,...
  77, 27, 50, 0];

config.safe_max_deg = [ ...
    115, 160, 140, 180,...
  104, 160, 140, 180,...
  180, 113, 125, 143,...
  113, 147, 150, 180];

% Channels actuated by the 2-DOF gait:
%   FL -> channels 10,11
%   FR -> channels 5,6
%   RL -> channels 13,14
%   RR -> channels 1,2
config.map.hip_fl.channel = 10;
config.map.hip_fl.model_column = "hip_fl_deg";
config.map.knee_fl.channel = 11;
config.map.knee_fl.model_column = "knee_fl_deg";

config.map.hip_fr.channel = 5;
config.map.hip_fr.model_column = "hip_fr_deg";
config.map.knee_fr.channel = 6;
config.map.knee_fr.model_column = "knee_fr_deg";

config.map.hip_rl.channel = 13;
config.map.hip_rl.model_column = "hip_rl_deg";
config.map.knee_rl.channel = 14;
config.map.knee_rl.model_column = "knee_rl_deg";

config.map.hip_rr.channel = 1;
config.map.hip_rr.model_column = "hip_rr_deg";
config.map.knee_rr.channel = 2;
config.map.knee_rr.model_column = "knee_rr_deg";

% Scaling from model delta angle to real-servo delta angle.
config.scale = struct( ...
    'hip_fl', 1.0, 'knee_fl', 1.0, ...
    'hip_fr', 1.0, 'knee_fr', 1.0, ...
    'hip_rl', 1.0, 'knee_rl', 1.0, ...
    'hip_rr', 1.0, 'knee_rr', 1.0);

% Optional sequence preparation.
config.sample_time = 0.05;
config.start_time = [];
config.end_time = [];
end
