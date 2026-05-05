function streamServoSequenceBluetooth(sequenceCsv, deviceName, sendDelay)
%STREAMSERVOSEQUENCEBLUETOOTH Send servo sequence rows to ESP32 over Bluetooth.
%   STREAMSERVOSEQUENCEBLUETOOTH() sends esp32_servo_sequence.csv to the
%   Bluetooth Classic device named "QuadrupedESP32".
%
%   STREAMSERVOSEQUENCEBLUETOOTH(sequenceCsv, deviceName, sendDelay)
%   lets you choose the sequence file, device name, and inter-packet delay
%   in seconds.
%
%   Requires MATLAB support for bluetooth() on the host machine.

if nargin < 1 || strlength(string(sequenceCsv)) == 0
    sequenceCsv = "esp32_servo_sequence.csv";
end

if nargin < 2 || strlength(string(deviceName)) == 0
    deviceName = "QuadrupedESP32";
end

if nargin < 3 || isempty(sendDelay)
    sendDelay = [];
end

sequenceCsv = string(sequenceCsv);
deviceName = string(deviceName);

seq = readtable(sequenceCsv);
required = ["time_ms", compose("ch%d", 0:15)];
missing = setdiff(required, string(seq.Properties.VariableNames));
if ~isempty(missing)
    error("Faltan columnas en la secuencia: %s", strjoin(missing, ", "));
end

if isempty(sendDelay)
    if height(seq) >= 2
        sendDelay = double(seq.time_ms(2) - seq.time_ms(1)) / 1000;
    else
        sendDelay = 0.05;
    end
end

bt = bluetooth(deviceName, 1);
cleanupObj = onCleanup(@() clear("bt"));

pause(2.0);

header = "time_ms," + strjoin(compose("ch%d", 0:15), ",");
writeline(bt, header);
pause(0.1);

for row = 1:height(seq)
    values = zeros(1, 17);
    values(1) = seq.time_ms(row);
    for ch = 0:15
        values(ch + 2) = seq.(sprintf("ch%d", ch))(row);
    end

    packet = strjoin(string(values), ",");
    writeline(bt, packet);
    pause(sendDelay);
end
end
