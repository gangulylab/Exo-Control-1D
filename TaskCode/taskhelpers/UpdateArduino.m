function [command,posX,posY] = UpdateArduino(Arduino,command,velX,velY)

if ~exist('command','var'),
    command = 0;
end
if ~exist('velX','var'),
    velX = 0;
end
if ~exist('velY','var')
    velY = 0;
end


% fprintf('Sent Vels: %03.02f, %03.02f\n',velX,velY);
xVelVal     = round(Arduino.vel.f_speed2bits(velX));
yVelVal     = round(Arduino.vel.f_speed2bits(velY));

% fprintf('Sent Vel Vals: %04.02i, %04.02i\n',xVelVal,yVelVal);
[command,xPosVal,yPosVal] = f_commBBS(Arduino.devBBS,command,xVelVal,yVelVal);
% fprintf('Received Pos Vals: %04.02i, %04.02i\n',xPosVal,yPosVal);

posX = Arduino.pos.f_bits2pos(xPosVal);
posY = Arduino.pos.f_bits2pos(yPosVal);
% fprintf('Received Pos: %03.02f, %03.02f\n',posX,posY);

end