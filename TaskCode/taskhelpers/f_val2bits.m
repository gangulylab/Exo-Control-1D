function [bits]  = f_val2bits(val,minSpeed,maxSpeed,minVoltage,maxVoltage,supplyVoltage,bitsDAC)

% Convert Vals to Voltages
if abs(val)<minSpeed
    val     = 0;
end
if abs(val)>maxSpeed
    val     = sign(val)*maxSpeed;
end
volts = val./(maxSpeed)*((maxVoltage-minVoltage)/2)+(maxVoltage+minVoltage)/2;

% Convert Voltages to bits
if volts<minVoltage
    volts     = minVoltage;
end
if volts>maxVoltage
    volts     = maxVoltage;
end
bits = volts./supplyVoltage*2^bitsDAC;

end