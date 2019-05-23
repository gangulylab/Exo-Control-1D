function VelocityArduino(Arduino,velX,velY)
% reset digital pins
if ~exist('velX','var'),
    velX = 0;
end
if ~exist('velY','var'),
    velY = 0;
end
inputValX   = Arduino.vel.fun(velX);
inputValY   = Arduino.vel.fun(velY);

% fprintf('Vel: %02.02f,\tBit: %04.0f\n',velX,inputValX);

inputValX  = (typecast(uint16(inputValX), 'uint8'));
write(Arduino.vel.XAddress,[inputValX(2) , inputValX(1)],'uint8')

inputValY  = (typecast(uint16(inputValY), 'uint8'));
write(Arduino.vel.YAddress,[inputValY(2) , inputValX(1)],'uint8')

end