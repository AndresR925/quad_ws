function servoTable = prepareServoCommands(inputCsv, outputCsv, config)
%PREPARESERVOCOMMANDS Convert joint trajectories to ESP32-friendly servo commands.
%   servoTable = PREPARESERVOCOMMANDS() reads quadruped_joint_angles.csv,
%   applies per-servo calibration, and exports:
%       - quadruped_servo_commands.csv
%       - quadruped_servo_commands.mat
%
%   servoTable = PREPARESERVOCOMMANDS(inputCsv, outputCsv, config)
%   lets you choose the source CSV, target CSV, and calibration config.
%
%   The output table contains one row per command sample with:
%       - time_s
%       - 8 calibrated servo angles in degrees
%       - 8 integer command values rounded for transmission
%
%   Typical usage:
%       cfg = defaultServoCommandConfig();
%       cfg.hip_fl.offset_deg = 90;
%       cfg.hip_fl.invert = true;
%       servoTable = prepareServoCommands("mi_marcha.csv", ...
%           "marcha_esp32.csv", cfg);

if nargin < 1 || strlength(string(inputCsv)) == 0
    inputCsv = "quadruped_joint_angles.csv";
end

if nargin < 2 || strlength(string(outputCsv)) == 0
    outputCsv = "quadruped_servo_commands.csv";
end

if nargin < 3 || isempty(config)
    config = defaultServoCommandConfig();
end

inputCsv = string(inputCsv);
outputCsv = string(outputCsv);
outputMat = replace(outputCsv, ".csv", ".mat");
if outputMat == outputCsv
    outputMat = outputCsv + ".mat";
end

jointTable = readtable(inputCsv);
localValidateJointTable(jointTable);

servoNames = localServoNames();
jointColumns = localJointColumns();

servoTable = table(jointTable.time_s, 'VariableNames', {'time_s'});

for idx = 1:numel(servoNames)
    servoName = servoNames{idx};
    jointColumn = jointColumns{idx};
    rawDeg = jointTable.(jointColumn);
    calibratedDeg = localApplyServoCalibration(rawDeg, config.(servoName));
    calibratedDeg = localResampleIfNeeded(calibratedDeg, numel(rawDeg));

    servoTable.([servoName '_deg']) = calibratedDeg;
    servoTable.([servoName '_cmd']) = int16(round(calibratedDeg));
end

servoTable = localResampleTableIfNeeded(servoTable, config);

writetable(servoTable, outputCsv);
save(outputMat, "servoTable", "config");

fprintf("CSV exportado en: %s\n", outputCsv);
fprintf("MAT exportado en: %s\n", outputMat);
fprintf("Muestras exportadas: %d\n", height(servoTable));
end

function localValidateJointTable(jointTable)
requiredColumns = ["time_s", ...
    "hip_fl_deg", "knee_fl_deg", "hip_fr_deg", "knee_fr_deg", ...
    "hip_rl_deg", "knee_rl_deg", "hip_rr_deg", "knee_rr_deg"];

missing = setdiff(requiredColumns, string(jointTable.Properties.VariableNames));
if ~isempty(missing)
    error("Faltan columnas en el CSV de entrada: %s", strjoin(missing, ", "));
end
end

function servoNames = localServoNames()
servoNames = { ...
    'hip_fl', 'knee_fl', ...
    'hip_fr', 'knee_fr', ...
    'hip_rl', 'knee_rl', ...
    'hip_rr', 'knee_rr'};
end

function jointColumns = localJointColumns()
jointColumns = { ...
    'hip_fl_deg', 'knee_fl_deg', ...
    'hip_fr_deg', 'knee_fr_deg', ...
    'hip_rl_deg', 'knee_rl_deg', ...
    'hip_rr_deg', 'knee_rr_deg'};
end

function calibratedDeg = localApplyServoCalibration(rawDeg, servoCfg)
calibratedDeg = rawDeg(:);

if servoCfg.invert
    calibratedDeg = -calibratedDeg;
end

calibratedDeg = servoCfg.scale * calibratedDeg;
calibratedDeg = calibratedDeg + servoCfg.offset_deg;
calibratedDeg = min(max(calibratedDeg, servoCfg.min_deg), servoCfg.max_deg);
end

function values = localResampleIfNeeded(values, expectedCount)
if numel(values) ~= expectedCount
    values = reshape(values, expectedCount, 1);
end
end

function servoTable = localResampleTableIfNeeded(servoTable, config)
timeVector = servoTable.time_s(:);

if isempty(timeVector)
    return;
end

if ~isempty(config.start_time)
    keepMask = timeVector >= config.start_time;
    servoTable = servoTable(keepMask, :);
    timeVector = servoTable.time_s(:);
end

if ~isempty(config.end_time)
    keepMask = timeVector <= config.end_time;
    servoTable = servoTable(keepMask, :);
    timeVector = servoTable.time_s(:);
end

if isempty(config.sample_time)
    return;
end

newTime = (timeVector(1):config.sample_time:timeVector(end))';
if isempty(newTime) || numel(newTime) == numel(timeVector)
    return;
end

resampled = table(newTime, 'VariableNames', {'time_s'});
valueColumns = servoTable.Properties.VariableNames(2:end);

for idx = 1:numel(valueColumns)
    columnName = valueColumns{idx};
    originalValues = double(servoTable.(columnName));
    interpolated = interp1(timeVector, originalValues, newTime, 'linear');

    if endsWith(columnName, "_cmd")
        resampled.(columnName) = int16(round(interpolated));
    else
        resampled.(columnName) = interpolated;
    end
end

servoTable = resampled;
end
