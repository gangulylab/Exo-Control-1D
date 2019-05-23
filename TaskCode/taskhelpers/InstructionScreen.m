function InstructionScreen(Params,tex)
% Display text then wait for subject to resume experiment

[command,posX,posY]             = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command          = command;
Params.Arduino.pos.planarPos    = [posX;posY];
% writeDigitalPin(Params.ArduinoPtr, Params.ArduinoTargetPins{1}, 0); % make sure the pin is at 0
% writeDigitalPin(Params.ArduinoPtr, Params.ArduinoTargetPins{2}, 0); % make sure the pin is at 0

% Pause Screen
DrawFormattedText(Params.WPTR, tex,'center','center',255);
Screen('Flip', Params.WPTR);

WaitSecs(.1);

while (1) % pause until subject presses spacebar to continue
    [~, ~, keyCode, ~] = KbCheck;
    if keyCode(KbName('space'))==1,
        keyCode(KbName('space'))=0;
        break;
    elseif keyCode(KbName('escape'))==1,
        ExperimentStop(1,Params);
    end
end

Screen('Flip', Params.WPTR);
WaitSecs(.1);

end % InstructionScreen