function exportEsp32SequenceHeader(sequenceCsv, outputHeader, variableName)
%EXPORTESP32SEQUENCEHEADER Export servo sequence CSV as a C header for ESP32.
%   EXPORTESP32SEQUENCEHEADER() converts esp32_servo_sequence.csv into
%   esp32_servo_sequence.h using the variable name gaitSequence.
%
%   The generated header contains:
%       - kGaitRows
%       - kGaitCols
%       - gaitSequence[row][col]
%
%   Column 1 is time_ms and columns 2..17 are ch0..ch15.

if nargin < 1 || strlength(string(sequenceCsv)) == 0
    sequenceCsv = "esp32_servo_sequence.csv";
end

if nargin < 2 || strlength(string(outputHeader)) == 0
    outputHeader = "esp32_servo_sequence.h";
end

if nargin < 3 || strlength(string(variableName)) == 0
    variableName = "gaitSequence";
end

sequenceCsv = string(sequenceCsv);
outputHeader = string(outputHeader);
variableName = string(variableName);

seq = readtable(sequenceCsv);
required = ["time_ms", compose("ch%d", 0:15)];
missing = setdiff(required, string(seq.Properties.VariableNames));
if ~isempty(missing)
    error("Faltan columnas en la secuencia: %s", strjoin(missing, ", "));
end

data = zeros(height(seq), 17);
data(:, 1) = seq.time_ms;
for ch = 0:15
    data(:, ch + 2) = seq.(sprintf("ch%d", ch));
end
data = round(data);

guardName = upper(regexprep(char(outputHeader), '[^A-Za-z0-9]', '_'));

lines = strings(0, 1);
lines(end + 1) = "#ifndef " + guardName;
lines(end + 1) = "#define " + guardName;
lines(end + 1) = "";
lines(end + 1) = "#include <stdint.h>";
lines(end + 1) = "";
lines(end + 1) = "static const uint16_t kGaitRows = " + height(seq) + ";";
lines(end + 1) = "static const uint16_t kGaitCols = 17;";
lines(end + 1) = "";
lines(end + 1) = "static const uint16_t " + variableName + "[kGaitRows][kGaitCols] = {";

for row = 1:size(data, 1)
    rowValues = join(string(data(row, :)), ", ");
    suffix = ",";
    if row == size(data, 1)
        suffix = "";
    end
    lines(end + 1) = "    {" + rowValues + "}" + suffix;
end

lines(end + 1) = "};";
lines(end + 1) = "";
lines(end + 1) = "#endif";

fid = fopen(outputHeader, "w");
if fid == -1
    error("No se pudo crear el archivo de salida: %s", outputHeader);
end

cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "%s\n", lines);

fprintf("Header exportado en: %s\n", outputHeader);
fprintf("Filas exportadas: %d\n", height(seq));
end
