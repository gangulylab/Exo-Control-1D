function [command,xPosVal,yPosVal] = f_commBBS(devBBS,command,xVelVal,yVelVal)
% Communicates with BBS server
% Sends desired velocities, and recieves actual planar positions



commandByte     = bitshift(command.planarEnable,5)...
                    +bitshift(command.velocityMode,4)...
                    +bitshift(command.target,0);

sendXVel    = typecast(bitshift(uint16(xVelVal),4), 'uint8');
sendYVel    = typecast(uint16(yVelVal), 'uint8');

byte1   = commandByte;
byte2   = sendXVel(2);
byte3   = sendYVel(2)+sendXVel(1);
byte4   = sendYVel(1);

write(devBBS,[byte1,byte2,byte3,byte4])

%------------------
% Read in Planar positions
readBytes   = read(devBBS,4);
commandByte = readBytes(1);

% command.filler          % 2-bit     Padding Zeros
% command.planarReady     % 1-bit     Turn planar off and on
% command.filler          % 5-bits    Padding Zeros

command.planarReady     = bitget(commandByte,6);

xPosHigh  = readBytes(2);
xPosLow   = bitand(readBytes(3), 240);
yPosHigh  = bitand(readBytes(3), 15);
yPosLow   = readBytes(4);

xPos    = typecast(uint8([xPosLow,xPosHigh]), 'uint16');
xPosVal    = bitshift(xPos,-4);
yPosVal    = typecast(uint8([yPosLow,yPosHigh]), 'uint16');
end