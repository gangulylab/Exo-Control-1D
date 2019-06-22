function ExperimentStop(fromPause,Params)
if ~exist('fromPause', 'var'), fromPause = 0; end

% Put the planar into a safe state
Params.Arduino.planar.enable        = 1;    % Enable Planar
Params.Arduino.planar.velocityMode  = 0;    % Set to position mode
Params.Arduino.planar.target        = 0;    % Set target as home
Params.Arduino = UpdateArduino(Params.Arduino);
pause(1)
while Params.Arduino.planar.ready == 0
    Params.Arduino = UpdateArduino(Params.Arduino);
    pause(0.5)
end
Params.Arduino.planar.enable        = 0;    % Disable Planar
Params.Arduino.glove.enable         = 0;
Params.Arduino = UpdateArduino(Params.Arduino);

% Close Screen
Screen('CloseAll');

% Close Serial Port
fclose('all');

% quit
fprintf('Ending Experiment\n')
if fromPause, keyboard; end

end % ExperimentStop
