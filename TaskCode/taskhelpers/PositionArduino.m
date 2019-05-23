function [Params] = PositionArduino(Params)
% voltX   = readVoltage(Params.ArduinoPtr,'A2');
voltX   = readVoltage(Params.ArduinoPtr,'A0');
% voltY   = readVoltage(Params.ArduinoPtr,'A2');
voltY   = readVoltage(Params.ArduinoPtr,'A1');

% fprintf('voltX: %04.02f,\t voltY: %04.02f mm\n', voltX, voltY);

voltX   = (voltX-Params.Arduino.pos.minVoltage)...
                ./(Params.Arduino.pos.maxVoltage-Params.Arduino.pos.minVoltage);
posX    = (voltX-0.5)*(Params.Arduino.pos.maxPos-Params.Arduino.pos.minPos);
voltY   = (voltY-Params.Arduino.pos.minVoltage)...
                ./(Params.Arduino.pos.maxVoltage-Params.Arduino.pos.minVoltage);
posY    = (voltY-0.5)*(Params.Arduino.pos.maxPos-Params.Arduino.pos.minPos);

Params.Arduino.pos.planarPos    = [posX,posY];



% outputVelX  = f_volts2pos(readVoltage(a,'A0'),minSpeed,maxSpeed,minVoltage,maxVoltage);
% outputVelY  = f_volts2pos(readVoltage(a,'A1'),minSpeed,maxSpeed,minVoltage,maxVoltage);


end