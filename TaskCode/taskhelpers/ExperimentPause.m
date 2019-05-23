function [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data)
% Display text then wait for subject to resume experiment

global Cursor

% Pause Screen
tex = 'Paused... Press ''p'' to continue, ''escape'' to quit, or ''d'' to debug';
DrawFormattedText(Params.WPTR, tex,'center','center',255);
Screen('Flip', Params.WPTR);

% set velocity on exo to 0
Params.Arduino.command.planarEnable     = 0;    % DisablePlanar
[command,posX,posY] = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command = command;

% add event to data structure
Data.Events(end+1).Time = GetSecs;
Data.Events(end).Str  = 'Pause';
if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'P0'); end
if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end

KbCheck;
WaitSecs(.1);
while 1, % pause until subject presses p again or quits
    [~, ~, keyCode, ~] = KbCheck;
    if keyCode(KbName('p'))==1,
        keyCode(KbName('p'))=0; % set to 0 to avoid multiple pauses in a row
        fprintf('\b') % remove input keys
        Params.Arduino.command.planarEnable     = 1;    % Enable Planar
        [command,posX,posY] = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
        Params.Arduino.command = command;
        break;
    end
    if keyCode(KbName('escape'))==1 || keyCode(KbName('q'))==1,
        ExperimentStop(1,Params); % quit experiment
    end
    if keyCode(KbName('d'))==1,
        keyboard; % quit experiment
    end
    
    % grab and process neural data
    tim = GetSecs;
    if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
        Cursor.LastUpdateTime = tim;
        Cursor.LastPredictTime = tim;
        if Params.BLACKROCK,
            [Neuro,Data] = NeuroPipeline(Neuro,Data);
            Data.NeuralTime(1,end+1) = tim;
        elseif Params.GenNeuralFeaturesFlag,
            Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
            Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
            Data.NeuralTime(1,end+1) = tim;
        end
    end
end

% add event to data structure
Data.Events(end+1).Time = GetSecs;
Data.Events(end).Str  = 'EndPause';
if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'PF'); end
if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end

Screen('Flip', Params.WPTR);
WaitSecs(.1);

end % ExperimentPause