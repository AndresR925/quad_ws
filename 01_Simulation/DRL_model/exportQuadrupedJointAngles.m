function jointTable = exportQuadrupedJointAngles(outputCsv)
%EXPORTQUADRUPEDJOINTANGLES Simulate the pretrained agent and export joint angles.
%   jointTable = EXPORTQUADRUPEDJOINTANGLES() runs the quadruped example
%   with the pretrained DDPG policy and exports the eight joint-angle
%   trajectories to:
%       - quadruped_joint_angles.csv
%       - quadruped_joint_angles.mat
%
%   jointTable = EXPORTQUADRUPEDJOINTANGLES(outputCsv) exports the CSV to
%   the file specified by outputCsv. A MAT file with the same base name is
%   also created.
%
%   The exported table contains one row per simulation step and includes
%   the joint references both in radians and degrees. These trajectories
%   are reconstructed from the observation vector produced by the Simulink
%   environment.

if nargin < 1 || strlength(string(outputCsv)) == 0
    outputCsv = "quadruped_joint_angles.csv";
end

outputCsv = string(outputCsv);
outputMat = replace(outputCsv, ".csv", ".mat");
if outputMat == outputCsv
    outputMat = outputCsv + ".mat";
end

previousRngState = rng(0, "twister");
cleanupRng = onCleanup(@() rng(previousRngState));

initializeRobotParameters;

mdl = "rlQuadrupedRobot";
blk = mdl + "/RL Agent";
load_system(mdl);

numObs = 44;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = "observations";

numAct = 8;
actInfo = rlNumericSpec([numAct 1], LowerLimit=-1, UpperLimit=1);
actInfo.Name = "torque";

env = rlSimulinkEnv(mdl, blk, obsInfo, actInfo);

agentOptions = rlDDPGAgentOptions();
agentOptions.SampleTime = Ts;
agentOptions.MiniBatchSize = 256;
agentOptions.ExperienceBufferLength = 1e6;
agentOptions.MaxMiniBatchPerEpoch = 200;
agentOptions.ActorOptimizerOptions.LearnRate = 1e-3;
agentOptions.ActorOptimizerOptions.GradientThreshold = 1;
agentOptions.CriticOptimizerOptions.LearnRate = 1e-3;
agentOptions.CriticOptimizerOptions.GradientThreshold = 1;
agentOptions.NoiseOptions.StandardDeviation = 0.1;
agentOptions.NoiseOptions.MeanAttractionConstant = 1.0;

initOpts = rlAgentInitializationOptions(NumHiddenUnit=256);
rng(0, "twister");
agent = rlDDPGAgent(obsInfo, actInfo, initOpts, agentOptions);

loaded = load("rlQuadrupedAgentParams.mat", "params");
setLearnableParameters(agent, loaded.params);

simOptions = rlSimulationOptions(MaxSteps=floor(Tf / Ts));
experience = sim(env, agent, simOptions);

[obsMatrix, timeVector] = localExtractObservationMatrix(experience, numObs, Ts);
jointTable = localBuildJointTable(obsMatrix, timeVector, ...
    q_hip_min, q_hip_max, q_knee_min, q_knee_max);

writetable(jointTable, outputCsv);
save(outputMat, "jointTable", "obsMatrix", "timeVector");

fprintf("CSV exportado en: %s\n", outputCsv);
fprintf("MAT exportado en: %s\n", outputMat);
fprintf("Muestras exportadas: %d\n", height(jointTable));
end

function jointTable = localBuildJointTable(obsMatrix, timeVector, qHipMin, qHipMax, qKneeMin, qKneeMax)
hipRange = qHipMax - qHipMin;
kneeRange = qKneeMax - qKneeMin;

flHip = localDenormalizeJoint(obsMatrix(13, :), qHipMin, hipRange);
flKnee = localDenormalizeJoint(obsMatrix(15, :), qKneeMin, kneeRange);
frHip = localDenormalizeJoint(obsMatrix(19, :), qHipMin, hipRange);
frKnee = localDenormalizeJoint(obsMatrix(21, :), qKneeMin, kneeRange);
rlHip = localDenormalizeJoint(obsMatrix(25, :), qHipMin, hipRange);
rlKnee = localDenormalizeJoint(obsMatrix(27, :), qKneeMin, kneeRange);
rrHip = localDenormalizeJoint(obsMatrix(31, :), qHipMin, hipRange);
rrKnee = localDenormalizeJoint(obsMatrix(33, :), qKneeMin, kneeRange);

jointTable = table( ...
    timeVector(:), ...
    flHip(:), flKnee(:), frHip(:), frKnee(:), ...
    rlHip(:), rlKnee(:), rrHip(:), rrKnee(:), ...
    rad2deg(flHip(:)), rad2deg(flKnee(:)), rad2deg(frHip(:)), rad2deg(frKnee(:)), ...
    rad2deg(rlHip(:)), rad2deg(rlKnee(:)), rad2deg(rrHip(:)), rad2deg(rrKnee(:)), ...
    'VariableNames', { ...
    'time_s', ...
    'hip_fl_rad', 'knee_fl_rad', 'hip_fr_rad', 'knee_fr_rad', ...
    'hip_rl_rad', 'knee_rl_rad', 'hip_rr_rad', 'knee_rr_rad', ...
    'hip_fl_deg', 'knee_fl_deg', 'hip_fr_deg', 'knee_fr_deg', ...
    'hip_rl_deg', 'knee_rl_deg', 'hip_rr_deg', 'knee_rr_deg'});
end

function joint = localDenormalizeJoint(qhat, qMin, qRange)
joint = ((qhat + 1) * 0.5 * qRange) + qMin;
end

function [obsMatrix, timeVector] = localExtractObservationMatrix(experience, numObs, Ts)
obsSource = localDigForObservation(experience);
timeVector = [];

if isnumeric(obsSource)
    obsMatrix = localReshapeObservation(obsSource, numObs);
elseif iscell(obsSource) && isscalar(obsSource)
    obsMatrix = localReshapeObservation(obsSource{1}, numObs);
elseif istimetable(obsSource)
    timeVector = seconds(obsSource.Properties.RowTimes - obsSource.Properties.RowTimes(1));
    obsMatrix = localReshapeObservation(obsSource.Variables.', numObs);
elseif isstruct(obsSource) && isfield(obsSource, "Data")
    obsMatrix = localReshapeObservation(obsSource.Data, numObs);
    if isfield(obsSource, "Time")
        timeVector = double(obsSource.Time(:));
    end
elseif isobject(obsSource) && isprop(obsSource, "Data")
    obsMatrix = localReshapeObservation(obsSource.Data, numObs);
    if isprop(obsSource, "Time")
        timeVector = double(obsSource.Time(:));
    end
else
    error("No fue posible extraer la matriz de observaciones de la simulacion.");
end

if isempty(timeVector)
    timeVector = (0:size(obsMatrix, 2)-1)' * Ts;
else
    timeVector = timeVector(:);
    if numel(timeVector) ~= size(obsMatrix, 2)
        timeVector = (0:size(obsMatrix, 2)-1)' * Ts;
    end
end
end

function obsSource = localDigForObservation(value)
obsSource = [];

if isnumeric(value) || iscell(value) || istimetable(value)
    obsSource = value;
    return;
end

if isstruct(value)
    if isfield(value, "Observation")
        obsSource = localDigForObservation(value.Observation);
        if ~isempty(obsSource)
            return;
        end
    end
    if isfield(value, "observations")
        obsSource = localDigForObservation(value.observations);
        if ~isempty(obsSource)
            return;
        end
    end
    if isfield(value, "Data")
        obsSource = value;
        return;
    end
    fields = fieldnames(value);
    for idx = 1:numel(fields)
        obsSource = localDigForObservation(value.(fields{idx}));
        if ~isempty(obsSource)
            return;
        end
    end
    return;
end

if isobject(value)
    if isprop(value, "Observation")
        obsSource = localDigForObservation(value.Observation);
        if ~isempty(obsSource)
            return;
        end
    end
    if isprop(value, "observations")
        obsSource = localDigForObservation(value.observations);
        if ~isempty(obsSource)
            return;
        end
    end
    if isprop(value, "Data")
        obsSource = value;
        return;
    end
end
end

function obsMatrix = localReshapeObservation(rawObs, numObs)
obsData = squeeze(rawObs);

if ~isnumeric(obsData)
    error("El contenedor de observaciones no es numerico.");
end

if isvector(obsData)
    if numel(obsData) ~= numObs
        error("El vector de observaciones no tiene %d elementos.", numObs);
    end
    obsMatrix = obsData(:);
    return;
end

obsSize = size(obsData);

if obsSize(1) == numObs
    obsMatrix = reshape(obsData, numObs, []);
    return;
end

if obsSize(end) == numObs
    permOrder = [ndims(obsData), 1:ndims(obsData)-1];
    obsMatrix = reshape(permute(obsData, permOrder), numObs, []);
    return;
end

error("No se pudo reorganizar la observacion a una matriz de %d filas.", numObs);
end
