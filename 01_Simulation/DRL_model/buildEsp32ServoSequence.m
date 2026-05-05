function esp32Table = buildEsp32ServoSequence(inputCsv, outputCsv, config)
%BUILDESP32SERVOSEQUENCE Convert MATLAB gait CSV to PCA9685 servo commands.
%   esp32Table = BUILDESP32SERVOSEQUENCE() reads quadruped_joint_angles.csv
%   and exports:
%       - esp32_servo_sequence.csv
%       - esp32_servo_sequence.mat
%
%   The resulting table contains:
%       - time_ms
%       - ch0 ... ch15 with angles in degrees for the PCA9685
%
%   The 2-DOF model drives muslo/canilla. The lateral cadera joints are
%   kept fixed at their rest positions for this first hardware mapping.

if nargin < 1 || strlength(string(inputCsv)) == 0
    inputCsv = "quadruped_joint_angles.csv";
end

if nargin < 2 || strlength(string(outputCsv)) == 0
    outputCsv = "esp32_servo_sequence.csv";
end

if nargin < 3 || isempty(config)
    config = physicalQuadrupedServoConfig();
end

inputCsv = string(inputCsv);
outputCsv = string(outputCsv);
outputMat = replace(outputCsv, ".csv", ".mat");
if outputMat == outputCsv
    outputMat = outputCsv + ".mat";
end

jointTable = readtable(inputCsv);
localValidateInput(jointTable, config);

jointTable = localTrimAndResample(jointTable, config);

channelMatrix = repmat(config.rest_deg(:).', height(jointTable), 1);
modelZero = jointTable(1, :);
jointNames = fieldnames(config.map);

for idx = 1:numel(jointNames)
    jointName = jointNames{idx};
    jointCfg = config.map.(jointName);
    channel = jointCfg.channel;

    modelDeg = jointTable.(jointCfg.model_column);
    modelZeroDeg = modelZero.(jointCfg.model_column);
    deltaDeg = modelDeg - modelZeroDeg;

    restDeg = config.rest_deg(channel + 1);
    dirSign = config.dir(channel + 1);
    scale = config.scale.(jointName);

    realDeg = restDeg + dirSign * scale * deltaDeg;
    realDeg = min(max(realDeg, config.safe_min_deg(channel + 1)), ...
        config.safe_max_deg(channel + 1));

    channelMatrix(:, channel + 1) = realDeg;
end

timeMs = round(1000 * jointTable.time_s(:));
esp32Table = table(timeMs, 'VariableNames', {'time_ms'});

for channel = 0:15
    esp32Table.(sprintf('ch%d', channel)) = round(channelMatrix(:, channel + 1));
end

writetable(esp32Table, outputCsv);
save(outputMat, "esp32Table", "config");

fprintf("CSV exportado en: %s\n", outputCsv);
fprintf("MAT exportado en: %s\n", outputMat);
fprintf("Muestras exportadas: %d\n", height(esp32Table));
end

function localValidateInput(jointTable, config)
requiredColumns = ["time_s"];
jointNames = fieldnames(config.map);
for idx = 1:numel(jointNames)
    requiredColumns(end + 1) = config.map.(jointNames{idx}).model_column; %#ok<AGROW>
end

missing = setdiff(requiredColumns, string(jointTable.Properties.VariableNames));
if ~isempty(missing)
    error("Faltan columnas en el CSV de entrada: %s", strjoin(missing, ", "));
end
end

function jointTable = localTrimAndResample(jointTable, config)
if ~isempty(config.start_time)
    jointTable = jointTable(jointTable.time_s >= config.start_time, :);
end

if ~isempty(config.end_time)
    jointTable = jointTable(jointTable.time_s <= config.end_time, :);
end

if isempty(config.sample_time) || height(jointTable) < 2
    return;
end

newTime = (jointTable.time_s(1):config.sample_time:jointTable.time_s(end))';
if numel(newTime) == height(jointTable)
    return;
end

resampled = table(newTime, 'VariableNames', {'time_s'});
valueColumns = jointTable.Properties.VariableNames(2:end);

for idx = 1:numel(valueColumns)
    col = valueColumns{idx};
    resampled.(col) = interp1(jointTable.time_s, jointTable.(col), newTime, 'linear');
end

jointTable = resampled;
end
