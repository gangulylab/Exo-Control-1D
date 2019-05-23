function [vel]  = f_volts2vels(volts,minSpeed,maxSpeed,minVoltage,maxVoltage)
% Voltage to Vals
vel = (volts-(minVoltage+maxVoltage)/2)/(maxVoltage-minVoltage)...
            *(2*maxSpeed);  
if abs(vel)>maxSpeed
    vel = sign(vel)*maxSpeed;
end
if abs(vel)<minSpeed
    vel = 0;
end
end