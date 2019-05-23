function [Data, Neuro, KF, Params] = RunTrial(Data,Params,Neuro,TaskFlag,KF)
% Runs a trial, saves useful data along the way
% Each trial contains the following pieces
% 1) Inter-trial interval
% 2) Get the cursor to the start target (center)
% 3) Hold position during an instructed delay period
% 4) Get the cursor to the reach target (different on each trial)
% 5) Feedback

global Cursor

%% Set up trial
StartTargetPos = Params.StartTargetPosition;
ReachTargetPos = Data.TargetPosition;

% Output to Command Line
fprintf('\nTrial: %i\n',Data.Trial)
fprintf('  Target: %i\n',Data.TargetPosition)
if Params.Verbose,
    if TaskFlag==2,
        fprintf('    Cursor Assistance: %i%%\n',round(100*Cursor.Assistance))
        if Params.CLDA.Type==3,
            %fprintf('    Lambda 1/2 life: %.2fsecs\n',log(.5)/log(KF.Lambda)/Params.UpdateRate)
            %fprintf('    Lambda 1/2 life: %.2fsecs\n',KF.Lambda)
        end
    end
end

% keep track of update times
dt_vec = [];
dT_vec = [];

%%==========================================================================================================================
% Move Planar system to the home position then disable it.
Params.Arduino.command.planarEnable     = 1;    % 1-bit     Enable Planar
Params.Arduino.command.velocityMode     = 0;    % 1-bit     Position Mode
Params.Arduino.command.target           = 0;    % 1-bit     Move to Home
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
pause(0.5)
command.planarReady = 0;
while command.planarReady == 0
    [command,posX,posY]         = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
    Params.Arduino.command      = command;
    [newX, newY, textHeight]=Screen('DrawText', Params.WPTR, 'Auto-correcting system to home position...',...
                    50, 50, [255,0,0], [0,0,0]);
    Screen('Flip', Params.WPTR);
    pause(0.25)
end
Cursor.State = [0,0,1]';
Params.Arduino.command.planarEnable     = 0;    % 1-bit     Disable Planar
[command,posX,posY]             = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command          = command;
Params.Arduino.pos.planarPos    = [posX;posY];

% grab blackrock data and run through processing pipeline
if Params.BLACKROCK,
    Cursor.LastPredictTime = GetSecs;
    Cursor.LastUpdateTime = Cursor.LastPredictTime;
    Neuro = NeuroPipeline(Neuro);
end



%% Inter Trial Interval
Data.ErrorID = 0;
if ~Data.ErrorID && Params.InterTrialInterval>0,
    tstart  = GetSecs;
    Data.Events(end+1).Time = tstart;
    Data.Events(end).Str  = 'Inter Trial Interval';
    if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'ITI'); end
    if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end
    
    if TaskFlag==1,
        OptimalCursorTraj = ...
            GenerateCursorTraj(Cursor.State(1),Cursor.State(1),Params.InterTrialInterval,Params);
        ct = 1;
    end
    
    done = 0;
    TotalTime = 0;
    while ~done,
        Screen('DrawText', Params.WPTR, 'Inter-trial Interval',50, 50, [255,0,0], [0,0,0]);
        % Update Time & Position
        tim = GetSecs;
        
        % for pausing and quitting expt
        if CheckPause, [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data); end
        
        % Update Screen Every Xsec
        if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
            % time
            dt = tim - Cursor.LastPredictTime;
            TotalTime = TotalTime + dt;
            dt_vec(end+1) = dt; %#ok<*AGROW>
            Cursor.LastPredictTime = tim;
            Data.Time(1,end+1) = tim;
            
            % grab and process neural data
            if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
                dT = tim-Cursor.LastUpdateTime;
                dT_vec(end+1) = dT;
                Cursor.LastUpdateTime = tim;
                if Params.BLACKROCK,
                    [Neuro,Data] = NeuroPipeline(Neuro,Data);
                    Data.NeuralTime(1,end+1) = tim;
                end
                if Params.GenNeuralFeaturesFlag,
                    Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                    if Params.BLACKROCK, % override
                        Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end) = tim;
                    else,
                        Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end+1) = tim;
                    end
                end
                if Neuro.DimRed.Flag,
                    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                    Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
                end
            end
            
            % cursor
            if TaskFlag==1, % imagined movements
                Cursor.State(2) = (OptimalCursorTraj(ct)'-Cursor.State(1))/dt;
                Cursor.State(1) = OptimalCursorTraj(ct);
                Cursor.Vcommand = Cursor.State(2);
                disp(Cursor.State')
                ct = ct + 1;
            end
            Data.CursorState(:,end+1) = Cursor.State;
            Data.IntendedCursorState(:,end+1) = Cursor.IntendedState;
            Data.CursorAssist(1,end+1) = Cursor.Assistance;
            
            CursorRect = Params.CursorRect;
            x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
            y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
            CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
            CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
            Screen('FillOval', Params.WPTR, ...
                cat(1,Params.CursorColor)', ...
                cat(1,CursorRect)')
            
            % Exo Position
            %             fprintf('Pos X: %04.02f,\tPos Y: %04.02f mm\n',...
            %                             Params.Arduino.pos.planarPos(1),...
            %                             Params.Arduino.pos.planarPos(2));
            planarRectangle = reshape(Params.Arduino.pos.planarBounds,2,2)...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            planarCirc      = reshape([-10,10,-10,10],2,2)...
                +[[0;0],Params.Arduino.pos.planarBounds(4).*[1;1]]...
                +kron([1,-1].*Params.Arduino.pos.planarPos',[1;1])...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            Screen('FrameRect', Params.WPTR, [100,0,0], planarRectangle([1,3,2,4]), [3]);
            Screen('FillOval', Params.WPTR, [100,0,0], planarCirc([1,3,2,4]), [3]);
            
            Screen('Flip', Params.WPTR);
        end
        
        % end if takes too long
        if TotalTime > Params.InterTrialInterval,
            done = 1;
        end
        
    end % Inter Trial Interval
end % only complete if no errors


%% Instructed Delay
% light led depending on target
idx = Data.TargetID;

% Map MATLAB Target Locations to Planar target positions
switch idx
    case 0
        Params.Arduino.command.target           = 0;
    case 1 % Left Target
        Params.Arduino.command.target           = 5; % Go West
    case 2 % Right Target
        Params.Arduino.command.target           = 1; % Go East
    otherwise
        Params.Arduino.command.target           = 0;
end
% Send updataed target params to system (should be in offline, postion ctrl mode)
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
Params.Arduino.pos.planarPos            = [posX;posY];

if ~Data.ErrorID && Params.InstructedDelayTime>0,
    tstart  = GetSecs;
    Data.Events(end+1).Time = tstart;
    Data.Events(end).Str  = 'Instructed Delay';
    if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'ID'); end
    if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end
    
    if TaskFlag==1,
        OptimalCursorTraj = ...
            GenerateCursorTraj(StartTargetPos,StartTargetPos,Params.InstructedDelayTime,Params);
        ct = 1;
    end
    
    done = 0;
    TotalTime = 0;
    InTargetTotalTime = 0;
    while ~done,        
        Screen('DrawText', Params.WPTR, 'Instruct to go target...',50, 50, [255,0,0], [0,0,0]);
        % Update Time & Position
        tim = GetSecs;
        
        % for pausing and quitting expt
        if CheckPause, [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data); end
        
        % Update Screen
        if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
            % time
            dt = tim - Cursor.LastPredictTime;
            TotalTime = TotalTime + dt;
            dt_vec(end+1) = dt;
            Cursor.LastPredictTime = tim;
            Data.Time(1,end+1) = tim;
            
            % grab and process neural data
            if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
                dT = tim-Cursor.LastUpdateTime;
                dT_vec(end+1) = dT;
                Cursor.LastUpdateTime = tim;
                if Params.BLACKROCK,
                    [Neuro,Data] = NeuroPipeline(Neuro,Data);
                    Data.NeuralTime(1,end+1) = tim;
                end
                if Params.GenNeuralFeaturesFlag,
                    Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                    if Params.BLACKROCK, % override
                        Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end) = tim;
                    else,
                        Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end+1) = tim;
                    end
                end
                if Neuro.DimRed.Flag,
                    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                    Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
                end
                %KF = UpdateCursor(Params,Neuro,TaskFlag,StartTargetPos,KF);
                %Params = PositionArduino(Params);
                %Cursor.State(1) = Params.Arduino.pos.planarPos;
            end
            
            % cursor
            if TaskFlag==1, % imagined movements
                Cursor.State(2) = (OptimalCursorTraj(ct)'-Cursor.State(1))/dt;
                Cursor.State(1) = OptimalCursorTraj(ct);
                Cursor.Vcommand = Cursor.State(2);
                ct = ct + 1;
            end
            CursorRect = Params.CursorRect;
            x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
            y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
            CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
            CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
            Data.CursorState(:,end+1) = Cursor.State;
            Data.PlanarState(:,end+1) = Params.Arduino.pos.planarPos(1);
            Data.IntendedCursorState(:,end+1) = Cursor.IntendedState;
            Data.CursorAssist(1,end+1) = Cursor.Assistance;
            
            % start target
            StartRect = Params.TargetRect; % centered at (0,0)
            x = StartTargetPos*cosd(Params.MvmtAxisAngle);
            y = StartTargetPos*sind(Params.MvmtAxisAngle);
            StartRect([1,3]) = StartRect([1,3]) + x + Params.Center(1); % add x-pos
            StartRect([2,4]) = StartRect([2,4]) + y + Params.Center(2); % add y-pos
            
            switch Params.Arduino.usePlanarAsCursor 
                case 0                    
                    inFlag = InTarget(Cursor,ReachTargetPos,Params.TargetSize);
                case 1 
                    foo.State(1) = Params.Arduino.pos.planarPos(1);
                    inFlag = InTarget(foo,ReachTargetPos,Params.TargetSize);
                    fprintf('Flag: %02.02f,\tCursor X: %03.03f,\tPlanar X: %03.03f\n',inFlag,x,foo.State(1));
            end    
            
            inFlag = InTarget(Cursor,StartTargetPos,Params.TargetSize);
            if inFlag, StartCol = Params.InTargetColor;
            else, StartCol = Params.OutTargetColor;
            end
            
            % reach target
            ReachRect = Params.TargetRect; % centered at (0,0)
            x = ReachTargetPos*cosd(Params.MvmtAxisAngle);
            y = ReachTargetPos*sind(Params.MvmtAxisAngle);
            ReachRect([1,3]) = ReachRect([1,3]) + x + Params.Center(1); % add x-pos
            ReachRect([2,4]) = ReachRect([2,4]) + y + Params.Center(2); % add y-pos
            ReachCol = Params.OutTargetColor;
            
            % draw
            Screen('FillOval', Params.WPTR, ...
                cat(1,ReachCol,Params.CursorColor)', ...
                cat(1,ReachRect,CursorRect)')
            %Screen('FillOval', Params.WPTR, ...
            %    cat(1,StartCol,ReachCol,Params.CursorColor)', ...
            %    cat(1,StartRect,ReachRect,CursorRect)')
            if Params.DrawVelCommand.Flag && TaskFlag>1,
                VelRect = Params.DrawVelCommand.Rect;
                VelRect([1,3]) = VelRect([1,3]) + Params.Center(1);
                VelRect([2,4]) = VelRect([2,4]) + Params.Center(2);
                x0 = mean(VelRect([1,3]));
                y0 = mean(VelRect([2,4]));
                xf = x0 + 0.1*Cursor.Vcommand*cosd(Params.MvmtAxisAngle);
                yf = y0 + 0.1*Cursor.Vcommand*sind(Params.MvmtAxisAngle);
                Screen('FrameOval', Params.WPTR, [100,100,100], VelRect);
                Screen('DrawLine', Params.WPTR, [100,100,100], x0, y0, xf, yf, 3);
            end
            
            % Exo Position
            %             fprintf('Pos X: %04.02f,\tPos Y: %04.02f mm\n',...
            %                             Params.Arduino.pos.planarPos(1),...
            %                             Params.Arduino.pos.planarPos(2));
            planarRectangle = reshape(Params.Arduino.pos.planarBounds,2,2)...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            planarCirc      = reshape([-10,10,-10,10],2,2)...
                +[[0;0],Params.Arduino.pos.planarBounds(4).*[1;1]]...
                +kron([1,-1].*Params.Arduino.pos.planarPos',[1;1])...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            Screen('FrameRect', Params.WPTR, [100,0,0], planarRectangle([1,3,2,4]), [3]);
            Screen('FillOval', Params.WPTR, [100,0,0], planarCirc([1,3,2,4]), [3]);
            
            Screen('DrawingFinished', Params.WPTR);
            Screen('Flip', Params.WPTR);
            
            % start counting time if cursor is in target
            if inFlag,
                InTargetTotalTime = InTargetTotalTime + dt;
            else, % error if they left too early
                done = 1;
                Data.ErrorID = 2;
                Data.ErrorStr = 'InstructedDelayHold';
                fprintf('ERROR: %s\n',Data.ErrorStr)
            end
        end
        
        % end if in start target for hold time
        if InTargetTotalTime > Params.InstructedDelayTime,
            done = 1;
        end
    end % Instructed Delay Loop
end % only complete if no errors


%% Go to reach target
% Enable planar into velocity control mode for reach from home to target
Params.Arduino.command.planarEnable     = 1;    % 1-bit     Move to target, or accept sent velocities
Params.Arduino.command.velocityMode     = 1;    % 1-bit     Move to target, or accept sent velocities
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
Params.Arduino.pos.planarPos            = [posX;posY];

Data.ErrorID = 0;
if ~Data.ErrorID,
    tstart  = GetSecs;
    Data.Events(end+1).Time = tstart;
    Data.Events(end).Str  = 'Reach Target';
    if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'RT'); end
    if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end
    
    if TaskFlag==1,
        OptimalCursorTraj = [...
            GenerateCursorTraj(StartTargetPos,ReachTargetPos,Params.ImaginedMvmtTime,Params);
            GenerateCursorTraj(ReachTargetPos,ReachTargetPos,Params.TargetHoldTime,Params)];
        ct = 1;
    end
    
    done = 0;
    TotalTime = 0;
    InTargetTotalTime = 0;
    while ~done,
        Screen('DrawText', Params.WPTR, 'Attempt to go to target...',50, 50, [255,0,0], [0,0,0]);
        % Update Time & Position
        tim = GetSecs;
        
        % for pausing and quitting expt
        if CheckPause, [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data); end
        
        % Update Screen
        if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
            % time
            dt = tim - Cursor.LastPredictTime;
            TotalTime = TotalTime + dt;
            dt_vec(end+1) = dt;
            Cursor.LastPredictTime = tim;
            Data.Time(1,end+1) = tim;
            
            % grab and process neural data
            if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
                dT = tim-Cursor.LastUpdateTime;
                dT_vec(end+1) = dT;
                Cursor.LastUpdateTime = tim;
                if Params.BLACKROCK,
                    [Neuro,Data] = NeuroPipeline(Neuro,Data);
                    Data.NeuralTime(1,end+1) = tim;
                end
                if Params.GenNeuralFeaturesFlag,
                    Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                    if Params.BLACKROCK, % override
                        Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end) = tim;
                    else,
                        Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end+1) = tim;
                    end
                end
                if Neuro.DimRed.Flag,
                    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                    Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
                end
                [KF,Params] = UpdateCursor(Params,Neuro,TaskFlag,ReachTargetPos,KF);
                %                 Params = PositionArduino(Params);
                %Cursor.State(1) = Params.Arduino.pos.planarPos;
            end
            
            % cursor
            if TaskFlag==1, % imagined movements
                disp(ct);
%                 disp(OptimalCursorTraj)
                Cursor.State(2) = (OptimalCursorTraj(ct)'-Cursor.State(1))/dt;
                Cursor.State(1) = OptimalCursorTraj(ct);
                Cursor.Vcommand = Cursor.State(2);
                ct = ct + 1;
            end
            CursorRect = Params.CursorRect;
                  
            x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
            y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
            CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
            CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
            Data.CursorState(:,end+1) = Cursor.State;
            Data.PlanarState(:,end+1) = Params.Arduino.pos.planarPos(1);
            Data.IntendedCursorState(:,end+1) = Cursor.IntendedState;
            Data.CursorAssist(1,end+1) = Cursor.Assistance;
            
            % reach target
            ReachRect = Params.TargetRect; % centered at (0,0)
            x = ReachTargetPos*cosd(Params.MvmtAxisAngle);
            y = ReachTargetPos*sind(Params.MvmtAxisAngle);
            ReachRect([1,3]) = ReachRect([1,3]) + x + Params.Center(1); % add x-pos
            ReachRect([2,4]) = ReachRect([2,4]) + y + Params.Center(2); % add y-pos
            
            switch Params.Arduino.usePlanarAsCursor 
                case 0                    
                    inFlag = InTarget(Cursor,ReachTargetPos,Params.TargetSize);
                case 1 
                    foo.State(1) = Params.Arduino.pos.planarPos(1);
                    inFlag = InTarget(foo,ReachTargetPos,Params.TargetSize);
            end    
            fprintf('Flag: %02.02f,\tCursor X: %03.03f,\tPlanar X: %03.03f\n',inFlag,x,foo.State(1));
            
            % draw
            if inFlag, ReachCol = Params.InTargetColor;
            else, ReachCol = Params.OutTargetColor;
            end
            Screen('FillOval', Params.WPTR, ...
                cat(1,ReachCol,Params.CursorColor)', ...
                cat(1,ReachRect,CursorRect)')
            if Params.DrawVelCommand.Flag && TaskFlag>1,
                VelRect = Params.DrawVelCommand.Rect;
                VelRect([1,3]) = VelRect([1,3]) + Params.Center(1);
                VelRect([2,4]) = VelRect([2,4]) + Params.Center(2);
                x0 = mean(VelRect([1,3]));
                y0 = mean(VelRect([2,4]));
                xf = x0 + 0.1*Cursor.Vcommand*cosd(Params.MvmtAxisAngle);
                yf = y0 + 0.1*Cursor.Vcommand*sind(Params.MvmtAxisAngle);
                Screen('FrameOval', Params.WPTR, [100,100,100], VelRect);
                Screen('DrawLine', Params.WPTR, [100,100,100], x0, y0, xf, yf, 3);
            end
            
            % Exo Position
            %             fprintf('Pos X: %04.02f,\tPos Y: %04.02f mm\n',...
            %                             Params.Arduino.pos.planarPos(1),...
            %                             Params.Arduino.pos.planarPos(2));
            planarRectangle = reshape(Params.Arduino.pos.planarBounds,2,2)...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            planarCirc      = reshape([-10,10,-10,10],2,2)...
                +[[0;0],Params.Arduino.pos.planarBounds(4).*[1;1]]...
                +kron([1,-1].*Params.Arduino.pos.planarPos',[1;1])...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            Screen('FrameRect', Params.WPTR, [100,0,0], planarRectangle([1,3,2,4]), [3]);
            Screen('FillOval', Params.WPTR, [100,0,0], planarCirc([1,3,2,4]), [3]);
            
            Screen('DrawingFinished', Params.WPTR);
            Screen('Flip', Params.WPTR);
            
            % start counting time if cursor is in target
            if inFlag,
                InTargetTotalTime = InTargetTotalTime + dt;
            else
                InTargetTotalTime = 0;
            end
        end
        
        % end if takes too long
        if TotalTime > Params.MaxReachTime,
            done = 1;
            Data.ErrorID = 3;
            Data.ErrorStr = 'ReachTarget';
            fprintf('ERROR: %s\n',Data.ErrorStr)
        end
        
        % end if in start target for hold time
        if InTargetTotalTime > Params.TargetHoldTime,
            done = 1;
        end
    end % Reach Target Loop
end % only complete if no errors


%% Move Planar to desired target then disable
Params.Arduino.command.planarEnable     = 1;    % 1-bit     Move to target, or accept sent velocities
Params.Arduino.command.velocityMode     = 0;    % 1-bit     Move to target, or accept sent velocities
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
pause(0.5)
command.planarReady = 0;
while command.planarReady == 0
    [command,posX,posY]         = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
    Params.Arduino.command      = command;
    [newX, newY, textHeight]=Screen('DrawText', Params.WPTR, 'Auto-correcting reach attempt...',...
                    50, 50, [255,0,0], [0,0,0]);
    Screen('Flip', Params.WPTR);
    pause(0.25)
end
Params.Arduino.command.planarEnable     = 0;    % 1-bit     Move to target, or accept sent velocities
Params.Arduino.command.velocityMode     = 0;    % 1-bit     Move to target, or accept sent velocities
[command,posX,posY]             = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command          = command;




%% Instructed Delay
Data.ErrorID = 0;
% light led depending To Go Home
Params.Arduino.command.target           = 0;
Params.Arduino.command.planarEnable     = 0;    % 1-bit     Move to target, or accept sent velocities
Params.Arduino.command.velocityMode     = 0;    % 1-bit     Move to target, or accept sent velocities
% Send updataed target params to system (should be in offline, postion ctrl mode)
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
Params.Arduino.pos.planarPos            = [posX;posY];


if ~Data.ErrorID && Params.InstructedDelayTime>0,
    tstart  = GetSecs;
    Data.Events(end+1).Time = tstart;
    Data.Events(end).Str  = 'Instructed Delay';
    if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'ID'); end
    if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end
    
    if TaskFlag==1,
        OptimalCursorTraj = ...
            GenerateCursorTraj(StartTargetPos,StartTargetPos,Params.InstructedDelayTime,Params);
        ct = 1;
    end
    
    done = 0;
    TotalTime = 0;
    InTargetTotalTime = 0;
    while ~done,        
        Screen('DrawText', Params.WPTR, 'Instruct to go home...',50, 50, [255,0,0], [0,0,0]);
        % Update Time & Position
        tim = GetSecs;
        
        % for pausing and quitting expt
        if CheckPause, [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data); end
        
        % Update Screen
        if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
            % time
            dt = tim - Cursor.LastPredictTime;
            TotalTime = TotalTime + dt;
            dt_vec(end+1) = dt;
            Cursor.LastPredictTime = tim;
            Data.Time(1,end+1) = tim;
            
            % grab and process neural data
            if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
                dT = tim-Cursor.LastUpdateTime;
                dT_vec(end+1) = dT;
                Cursor.LastUpdateTime = tim;
                if Params.BLACKROCK,
                    [Neuro,Data] = NeuroPipeline(Neuro,Data);
                    Data.NeuralTime(1,end+1) = tim;
                end
                if Params.GenNeuralFeaturesFlag,
                    Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                    if Params.BLACKROCK, % override
                        Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end) = tim;
                    else,
                        Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end+1) = tim;
                    end
                end
                if Neuro.DimRed.Flag,
                    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                    Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
                end
                %KF = UpdateCursor(Params,Neuro,TaskFlag,StartTargetPos,KF);
                %Params = PositionArduino(Params);
                %Cursor.State(1) = Params.Arduino.pos.planarPos;
            end
            
            % cursor
            if TaskFlag==1, % imagined movements
                Cursor.State(2) = (OptimalCursorTraj(ct)'-Cursor.State(1))/dt;
                Cursor.State(1) = OptimalCursorTraj(ct);
                Cursor.Vcommand = Cursor.State(2);
                ct = ct + 1;
            end
            CursorRect = Params.CursorRect;
            x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
            y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
            CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
            CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
            Data.CursorState(:,end+1) = Cursor.State;
            Data.PlanarState(:,end+1) = Params.Arduino.pos.planarPos(1);
            Data.IntendedCursorState(:,end+1) = Cursor.IntendedState;
            Data.CursorAssist(1,end+1) = Cursor.Assistance;
            
            % start target
            StartRect = Params.TargetRect; % centered at (0,0)
            x = StartTargetPos*cosd(Params.MvmtAxisAngle);
            y = StartTargetPos*sind(Params.MvmtAxisAngle);
            StartRect([1,3]) = StartRect([1,3]) + x + Params.Center(1); % add x-pos
            StartRect([2,4]) = StartRect([2,4]) + y + Params.Center(2); % add y-pos
            Screen('FillOval', Params.WPTR, ...
                cat(1,StartCol,Params.CursorColor)', ...
                cat(1,StartRect,CursorRect)')
            
            switch Params.Arduino.usePlanarAsCursor 
                case 0                    
                    inFlag = InTarget(Cursor,ReachTargetPos,Params.TargetSize);
                case 1 
                    foo.State(1) = Params.Arduino.pos.planarPos(1);
                    inFlag = InTarget(foo,ReachTargetPos,Params.TargetSize);
                    fprintf('Flag: %02.02f,\tCursor X: %03.03f,\tPlanar X: %03.03f\n',inFlag,x,foo.State(1));
            end    
            
            inFlag = InTarget(Cursor,StartTargetPos,Params.TargetSize);
            if inFlag, StartCol = Params.InTargetColor;
            else, StartCol = Params.OutTargetColor;
            end
            
            % reach target
            ReachRect = Params.TargetRect; % centered at (0,0)
            x = ReachTargetPos*cosd(Params.MvmtAxisAngle);
            y = ReachTargetPos*sind(Params.MvmtAxisAngle);
            ReachRect([1,3]) = ReachRect([1,3]) + x + Params.Center(1); % add x-pos
            ReachRect([2,4]) = ReachRect([2,4]) + y + Params.Center(2); % add y-pos
            ReachCol = Params.OutTargetColor;
            
            % draw
%             Screen('FillOval', Params.WPTR, ...
%                 cat(1,ReachCol,Params.CursorColor)', ...
%                 cat(1,ReachRect,CursorRect)')
            if Params.DrawVelCommand.Flag && TaskFlag>1,
                VelRect = Params.DrawVelCommand.Rect;
                VelRect([1,3]) = VelRect([1,3]) + Params.Center(1);
                VelRect([2,4]) = VelRect([2,4]) + Params.Center(2);
                x0 = mean(VelRect([1,3]));
                y0 = mean(VelRect([2,4]));
                xf = x0 + 0.1*Cursor.Vcommand*cosd(Params.MvmtAxisAngle);
                yf = y0 + 0.1*Cursor.Vcommand*sind(Params.MvmtAxisAngle);
                Screen('FrameOval', Params.WPTR, [100,100,100], VelRect);
                Screen('DrawLine', Params.WPTR, [100,100,100], x0, y0, xf, yf, 3);
            end
            
            % Exo Position
            %             fprintf('Pos X: %04.02f,\tPos Y: %04.02f mm\n',...
            %                             Params.Arduino.pos.planarPos(1),...
            %                             Params.Arduino.pos.planarPos(2));
            planarRectangle = reshape(Params.Arduino.pos.planarBounds,2,2)...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            planarCirc      = reshape([-10,10,-10,10],2,2)...
                +[[0;0],Params.Arduino.pos.planarBounds(4).*[1;1]]...
                +kron([1,-1].*Params.Arduino.pos.planarPos',[1;1])...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            Screen('FrameRect', Params.WPTR, [100,0,0], planarRectangle([1,3,2,4]), [3]);
            Screen('FillOval', Params.WPTR, [100,0,0], planarCirc([1,3,2,4]), [3]);
            
            Screen('DrawingFinished', Params.WPTR);
            Screen('Flip', Params.WPTR);
            
            % start counting time if cursor is in target
            if inFlag,
                InTargetTotalTime = InTargetTotalTime + dt;
            else, % error if they left too early
                done = 1;
                Data.ErrorID = 2;
                Data.ErrorStr = 'InstructedDelayHold';
                fprintf('ERROR: %s\n',Data.ErrorStr)
            end
        end
        
        % end if in start target for hold time
        if InTargetTotalTime > Params.InstructedDelayTime,
            done = 1;
        end
    end % Instructed Delay Loop
end % only complete if no errors



%% Go to Start Target
Data.ErrorID = 0;
% Update cursor position to match planar position
if Params.Arduino.usePlanarAsCursor == 1
    Cursor.State(1)     = Params.Arduino.pos.planarPos(1);
end
% Enable planar into velocity control mode for reach from target to home
Params.Arduino.command.planarEnable     = 1;    % 1-bit     Enable system
Params.Arduino.command.velocityMode     = 1;    % 1-bit     Allow control from Brain
[command,posX,posY]                     = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command                  = command;
Params.Arduino.pos.planarPos            = [posX;posY];


if ~Data.ErrorID %&& ~Params.CenterReset,
    tstart  = GetSecs;
    Data.Events(end+1).Time = tstart;
    Data.Events(end).Str  = 'Start Target';
    if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'ST'); end
    if Params.ArduinoSync, PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,length(Data.Events)); end
    
    if TaskFlag==1,
        OptimalCursorTraj = [...
            GenerateCursorTraj(Cursor.State(1),StartTargetPos,Params.ImaginedMvmtTime,Params);
            GenerateCursorTraj(StartTargetPos,StartTargetPos,Params.TargetHoldTime,Params)];
        ct = 1;
    end
    
    done = 0;
    TotalTime = 0;
    InTargetTotalTime = 0;
    while ~done,
        Screen('DrawText', Params.WPTR, 'Attempt to go home...',50, 50, [255,0,0], [0,0,0]);
        % Update Time & Position
        tim = GetSecs;
        
        % for pausing and quitting expt
        if CheckPause, [Neuro,Data,Params] = ExperimentPause(Params,Neuro,Data); end
        
        % Update Screen Every Xsec
        if (tim-Cursor.LastPredictTime) > 1/Params.ScreenRefreshRate,
            tic;
            % time
            dt = tim - Cursor.LastPredictTime;
            TotalTime = TotalTime + dt;
            dt_vec(end+1) = dt;
            Cursor.LastPredictTime = tim;
            Data.Time(1,end+1) = tim;
            
            % grab and process neural data
            if ((tim-Cursor.LastUpdateTime)>1/Params.UpdateRate),
                dT = tim-Cursor.LastUpdateTime;
                dT_vec(end+1) = dT;
                Cursor.LastUpdateTime = tim;
                if Params.BLACKROCK,
                    [Neuro,Data] = NeuroPipeline(Neuro,Data);
                    Data.NeuralTime(1,end+1) = tim;
                end
                if Params.GenNeuralFeaturesFlag,
                    Neuro.NeuralFeatures = VelToNeuralFeatures(Params);
                    if Params.BLACKROCK, % override
                        Data.NeuralFeatures{end} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end) = tim;
                    else,
                        Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
                        Data.NeuralTime(1,end+1) = tim;
                    end
                end
                if Neuro.DimRed.Flag,
                    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
                    Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
                end
                [KF,Params] = UpdateCursor(Params,Neuro,TaskFlag,StartTargetPos,KF);
            end
            
            % cursor
            if TaskFlag==1, % imagined movements
                Cursor.State(2) = (OptimalCursorTraj(ct)'-Cursor.State(1))/dt;
                Cursor.State(1) = OptimalCursorTraj(ct);
                Cursor.Vcommand = Cursor.State(2);
                ct = ct + 1;
            end
            
            CursorRect = Params.CursorRect;
            x = Cursor.State(1)*cosd(Params.MvmtAxisAngle);
            y = Cursor.State(1)*sind(Params.MvmtAxisAngle);
            CursorRect([1,3]) = CursorRect([1,3]) + x + Params.Center(1); % add x-pos
            CursorRect([2,4]) = CursorRect([2,4]) + y + Params.Center(2); % add y-pos
            Data.CursorState(:,end+1) = Cursor.State;
            Data.PlanarState(:,end+1) = Params.Arduino.pos.planarPos(1);
            Data.IntendedCursorState(:,end+1) = Cursor.IntendedState;
            Data.CursorAssist(1,end+1) = Cursor.Assistance;
            
            % start target
            StartRect = Params.TargetRect; % centered at (0,0)
            x = StartTargetPos*cosd(Params.MvmtAxisAngle);
            y = StartTargetPos*sind(Params.MvmtAxisAngle);
            StartRect([1,3]) = StartRect([1,3]) + x + Params.Center(1); % add x-pos
            StartRect([2,4]) = StartRect([2,4]) + y + Params.Center(2); % add y-pos
            
            switch Params.Arduino.usePlanarAsCursor 
                case 0                    
                    inFlag = InTarget(Cursor,StartTargetPos,Params.TargetSize);
                case 1 
                    foo.State(1) = Params.Arduino.pos.planarPos(1);
                    inFlag = InTarget(foo,StartTargetPos,Params.TargetSize);
                    fprintf('Flag: %02.02f,\tCursor X: %03.03f,\tPlanar X: %03.03f\n',inFlag,x,foo.State(1));
            end    
            
            if inFlag, StartCol = Params.InTargetColor;
            else, StartCol = Params.OutTargetColor;
            end
            
            % draw
            Screen('FillOval', Params.WPTR, ...
                cat(1,StartCol,Params.CursorColor)', ...
                cat(1,StartRect,CursorRect)')
            if Params.DrawVelCommand.Flag && TaskFlag>1,
                VelRect = Params.DrawVelCommand.Rect;
                VelRect([1,3]) = VelRect([1,3]) + Params.Center(1);
                VelRect([2,4]) = VelRect([2,4]) + Params.Center(2);
                x0 = mean(VelRect([1,3]));
                y0 = mean(VelRect([2,4]));
                xf = x0 + 0.1*Cursor.Vcommand*cosd(Params.MvmtAxisAngle);
                yf = y0 + 0.1*Cursor.Vcommand*sind(Params.MvmtAxisAngle);
                Screen('FrameOval', Params.WPTR, [100,100,100], VelRect);
                Screen('DrawLine', Params.WPTR, [100,100,100], x0, y0, xf, yf, 3);
            end
            
            % Exo Position
            %             fprintf('Pos X: %04.02f,\tPos Y: %04.02f mm\n',...
            %                             Params.Arduino.pos.planarPos(1),...
            %                             Params.Arduino.pos.planarPos(2));
            planarRectangle = reshape(Params.Arduino.pos.planarBounds,2,2)...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            planarCirc      = reshape([-10,10,-10,10],2,2)...
                +[[0;0],Params.Arduino.pos.planarBounds(4).*[1;1]]...
                +kron([1,-1].*Params.Arduino.pos.planarPos',[1;1])...
                +kron(Params.Arduino.pos.planarPlotLoc,ones(2,1));
            Screen('FrameRect', Params.WPTR, [100,0,0], planarRectangle([1,3,2,4]), [3]);
            Screen('FillOval', Params.WPTR, [100,0,0], planarCirc([1,3,2,4]), [3]);
            
            Screen('DrawingFinished', Params.WPTR);
            Screen('Flip', Params.WPTR);
            
            % start counting time if cursor is in target
            if inFlag,
                InTargetTotalTime = InTargetTotalTime + dt;
            else
                InTargetTotalTime = 0;
            end
        end
        
        % end if takes too long
        if TotalTime > Params.MaxStartTime,
            done = 1;
            Data.ErrorID = 1;
            Data.ErrorStr = 'StartTarget';
            fprintf('ERROR: %s\n',Data.ErrorStr)
        end
        
        % end if in start target for hold time
        if InTargetTotalTime > Params.TargetHoldTime,
            done = 1;
        end
    end % Start Target Loop
else % only complete if no errors and no automatic reset to center
    Cursor.State = [0,0,1]';
end
if Params.Arduino.usePlanarAsCursor == 1
    Cursor.State(1)     = Params.Arduino.pos.planarPos(1);
end

% On completipon of attempted return motion, disable planar and switch to position mode
Params.Arduino.command.planarEnable     = 0;    % 1-bit     Move to target, or accept sent velocities
Params.Arduino.command.velocityMode     = 0;    % 1-bit     Move to target, or accept sent velocities
[command,posX,posY]             = UpdateArduino(Params.Arduino,Params.Arduino.command,0,0);
Params.Arduino.command          = command;





%% Completed Trial - Give Feedback
Screen('Flip', Params.WPTR);

% output update times
if Params.Verbose,
    fprintf('      Screen Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.ScreenRefreshRate,mean(1./dt_vec),std(1./dt_vec))
    fprintf('      System Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.UpdateRate,mean(1./dT_vec),std(1./dT_vec))
end

% output feedback
if Data.ErrorID==0,
    fprintf('SUCCESS\n')
    if Params.FeedbackSound,
        sound(Params.RewardSound,Params.RewardSoundFs)
    end
else
    % reset cursor
    Cursor.State = [0,0,1]';
    Cursor.IntendedState = [0,0,1]';
    
    if Params.FeedbackSound,
        sound(Params.ErrorSound,Params.ErrorSoundFs)
    end
    WaitSecs(Params.ErrorWaitTime);
end

end % RunTrial



