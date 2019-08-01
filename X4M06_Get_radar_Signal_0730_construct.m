% This example demonstrates the use of the BasicRadarClassX4 example class
% to configure the chip and start streaming.
%
% To complete the following example you need:
% - An X4M200/X4M300/X4M03 module
% - The ModuleConnector library
% - MATLAB

%% Path
addModuleConnectorPath();
addpath('../../matlab/');
addpath('../../include/');
addpath('./');
% if running on a 32-bit Windows system, instead run:
% addModuleConnectorPath('win32');

clc
clear
close all

%% Activate parallel pool
% delete(gcp('nocreate'));
hPool = gcp('nocreate');
if isempty(hPool)
    parpool(4)
end

%% Settings
% Input parameters
COM = 'COM6';
FPS = 30;
dataType = 'rf';

% Chip settings
PPS = 35;
DACmin = 929;
DACmax = 1120;
Iterations = 64;
FrameStart = 0.3;
FrameStop = 1.94; % Sample length(in fast time) : 256

% Loopback filter setting
alpha = 0.96;

% Load the library
Lib = ModuleConnector.Library;
Lib.libfunctions

% Sample setting
Min_slow_index = 150;
Max_sample_length = 240;
Vital_Signal = [];

% Index
slow_index = 0;
index = 0;

% Assessment Var
Spent_time = [];
R_square_list = zeros(100, 256);

%% Init Radar

% Create BasicRadarClassX4 object
radar = BasicRadarClassX4(COM,FPS,dataType);

% Open radar.
radar.open();

% Use X4M300 interface to attempt to set sensor mode XEP (manual).
app = radar.mc.get_x4m300();

app.set_sensor_mode('stop');
try
    app.set_sensor_mode('XEP');
catch
    % Unable to set sensor mode. Assume only running XEP FW.
end

% Initialize radar.
radar.init();

% Configure X4 chip.
radar.radarInstance.x4driver_set_pulsesperstep(PPS);
radar.radarInstance.x4driver_set_dac_min(DACmin);
radar.radarInstance.x4driver_set_dac_max(DACmax);
radar.radarInstance.x4driver_set_iterations(Iterations);

% Configure frame area
radar.radarInstance.x4driver_set_frame_area(FrameStart,FrameStop);

% Read back actual set frame area
[frameStart, frameStop] = radar.radarInstance.x4driver_get_frame_area();

%% Start streaming and subscribe to message_data_float.
radar.start();

figure(1)

%% Receive Signal from Radar
while ishandle(1)
    % Peek message data float
    numPackets = radar.bufferSize(); 
    if numPackets > 0
        % Increase Index
        index = index + 1;
        slow_index = slow_index + 1;
        
        % Get frame (uses read_message_data_float)
        [frame, count] = radar.GetFrameNormalized(); 
        
        % Init
        if index == 1
            % Create figure
            figure(1)
            axh(1) = subplot(2,1,1);
            
            % Disp graph
            ph = plot(0);
            hold on
            pointh = plot(0);
            th = title(axh(1), '');
            hold off
            
            % Set graph details
            ylabel('Normalized amplitude');
            xlabel('Range [m]');
            grid on;
            
            % Length
            numBins = length(frame);
            if strcmp('bb', dataType)
                numBins = numBins/2;
            end

            % Fast time to Distance
            binLength = (frameStop-frameStart)/(numBins-1);
            rangeVec = (0:numBins-1)*binLength + frameStart;
            ph.XData = rangeVec;
            
            % Define
            Clutter = zeros(numBins,1);
            RawData = zeros(numBins,1);
            Signal = zeros(numBins, Max_sample_length);
            
%             Inphase_Signal = zeros(numBins, Max_sample_length);
%             Quadrature_Signal = zeros(numBins, Max_sample_length);
        end
        
       %% Pre-Processing(loopback filter)
        switch dataType
            case 'rf'
                % Loopback filter
                Clutter = alpha*Clutter+(1-alpha)*frame;
                RawData =  frame - Clutter;
                Signal(:,slow_index) = RawData;
                
                % Todo : Compensate small body or outside movement
                
                % Select max amplitude
                [max_amp, max_index] = max(abs(Signal(:,slow_index)));

                % Print graph
                ph.YData = Signal(:,slow_index);
                pointh.XData = rangeVec(max_index);
                pointh.YData = Signal(max_index,slow_index);
                pointh.Marker = 'o';
                pointh.MarkerEdgeColor = 'r';
                axh(1).YLim = [-0.005 0.005];
                
            case 'bb'
%                 Inphase = frame(1:end/2);
%                 Quadrature = abs(frame(end/2 + 1:end));
%                 
%                 % Loopback filter
%                 Inphase_Clutter = alpha*Inphase+(1-alpha)*Inphase;
%                 Qudarture_Clutter = alpha*Quadrature+(1-alpha)*Quadrature;
%                 Inphase_RawData =  Inphase - Inphase_Clutter;
%                 Qudarture_RawData = Quadrature - Qudarture_Clutter;
%                 
%                 Inphase_Signal(:,slow_index) = Inphase_RawData;
%                 Quadrature_Signal(:,slow_index) = Qudarture_RawData;
%                 
%                 Signal(:,slow_index) = abs(Inphase_RawData + Qudarture_RawData);
%                 
%                 ph.YData = abs(Inphase_RawData + Qudarture_RawData);
%                 ylim([-0.005 0.005]);
        end
        th.String = ['FrameNo: ' num2str(slow_index) ' / Length: ' num2str(length(frame)) ' / Frame drop : ' num2str(count - index)];
        drawnow;
    end
    
    % Remove first noise 
    if index == Min_slow_index
        slow_index = 0;
        RawData = zeros(numBins,1);
        Signal = zeros(numBins, Max_sample_length);
    end
    
    % Todo : If detect big body movement, ignore that radar signal
    
    %% Detect Vital Signal
    if slow_index >= Max_sample_length
        Sample = Signal(:, 1 : Max_sample_length);       
        % Wait until calc finished
        if exist('func_result', 'var')
            fprintf("Calc Delayed.\n");
            wait(func_result);
        else
            % Detect vital signal in parallel
            func_result = parfeval(@Detect_Vital_construct, 4, Sample, FPS);
            slow_index = 0;
        end
    end
    
    %% Show Detected Vital Signal
    if exist('func_result', 'var') && strcmp(func_result.State, 'finished') % Calc is finished
        % Get result
        [~, Selected_Signal, Selected_Index, error, info] = fetchNext(func_result);
        % Print result
        axh(2) = subplot(2,1,2);
        fprintf("%.1f s > ", (index - Min_slow_index)/FPS);
        if isempty(error)
            % reverse phase
            target = normalize(Selected_Signal);
            if isempty(Vital_Signal)
                Vital_Signal = target;
            elseif mean(Vital_Signal(end - 10 : end)) * mean(target(1 : 10)) < 0
                Vital_Signal = [Vital_Signal target];
            else
                Vital_Signal = [Vital_Signal -target];
            end
            
            fprintf("%.3f m Detected\n", Selected_Index * binLength + frameStart);
            
            % Draw graph
            plot(Vital_Signal);
            title(axh(2), ['Detected Vital Signal until ' num2str(length(Vital_Signal)/FPS) 's']);
            xlabel("Slow time index (bins)");
            ylabel("Normalized Amplitude");
            xlim(axh(2), [0 length(Vital_Signal)]);
            ylim(axh(2), [-5 5]);
            drawnow;
        else
            fprintf(error);
        end
        % Assessment var
        Spent_time = [Spent_time info.Time];
        R_square_list(length(Spent_time), :) = info.Rsquare;
        
        clear func_result;
        
        % Extract vital signal 
        if length(Vital_Signal) >= analysis_length
            target = Vital_Signal(end - analysis_length + 1 : end);
            % Extract Vital Signal by FFT
            [RR, HR, error] = Extract_Vital(target, FPS);
            % Print result
            if isempty(error)
                fprintf("Result > Respiration Rate : %.2f bpm / Heart Rate : ", RR);
                fprintf("%.2f bpm ", HR);
                fprintf("\n");
                
                % Save RR, HR
                RR_list = [RR_list RR];
                HR_list(length(RR_list), :) = HR;
            else
                fprintf(error);
            end
        end
    end
    
end

radar.close();

%% Show result
figure(3)
analysis_timeRange = (0 : length(RR_list))*Max_sample_length/FPS + analysis_length/FPS;
spent_timeRange = (0 : length(Spent_time))*Max_sample_length/FPS;
timeRange = (0 : length(Vital_Signal))/FPS;


hResult(1) = subplot(4,1,1);
title("RR in detected vital signal");
stairs(analysis_timeRange, RR_list)
xlabel("Slow time(s)")
ylabel("RR (bpm)")
ylim(hResult(1), [10 20])

hResult(2) = subplot(4,1,2);
title("HR in detected vital signal");
stairs(analysis_timeRange, HR_list(1))
hold on
stairs(analysis_timeRange, HR_list(2))
stairs(analysis_timeRange, HR_list(3))
hold off
xlabel("Slow time(s)")
ylabel("HR (bpm)")
ylim(hResult(2), [50 100])

hResult(3) = subplot(4,1,3);
title("Detected vital signal");
plot(timeRange, Vital_Signal)
xlabel("Slow time(s)")
ylabel("Normalized amplitude")
ylim(hResult(3), [5 5])

hResult(4) = subplot(4,1,4);
title("Spent time to detect vital signal");
stairs(spent_timeRange, Spent_time)
xlabel("Slow time(s)")
ylabel("Spent time(s)")
ylim(hResult(4), [0 20])
