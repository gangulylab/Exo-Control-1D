function ExperimentStop(fromPause,Params)
if ~exist('fromPause', 'var'), fromPause = 0; end

% Put the planar into a safe state
Params.Arduino.command.planarEnable     = 1;    % Enable Planar
Params.Arduino.command.velocityMode     = 0;    % Set to position mode
Params.Arduino.command.target           = 0;    % Set target as home
[command,posX,posY] = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command = command;
pause(0.1)
while command.planarReady == 0
    [command,posX,posY]         = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
    Params.Arduino.command      = command;
    pause(0.1)
end
Params.Arduino.command.planarEnable     = 0;    % Disable Planar
[command,posX,posY] = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);



% Close Screen
Screen('CloseAll');

% Close Serial Port
fclose('all');

% quit
fprintf('Ending Experiment\n')
if fromPause, keyboard; end

end % ExperimentStop
