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
Max_sample_length = 210;
Vital_Signal = [];

% Index
slow_index = 0;
index = 0;

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
            Signal = zeros(numBins, Min_slow_index);
            
%             Inphase_Signal = zeros(numBins, Max_sample_length);
%             Quadrature_Signal = zeros(numBins, Max_sample_length);
        end
        
       %% Pre-Processing(loopback filter)
        switch dataType
            case 'rf'
                % Loopback filter
                Clutter = alpha*Clutter+(1-alpha)*frame;
                RawData =  frame - Clutter;
%                 Signal(:,slow_index) = RawData;
                
                % Wavelet Denoise in fast time
                Filtered = wdenoise(RawData, 2, 'Wavelet', 'bior2.6', 'DenoisingMethod', 'SURE', 'NoiseEstimate', 'LevelIndependent');
                Signal(:,slow_index) = Filtered;
                
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
    
    %% Ignore Body Movement
    
    %% Detect Vital Signal
    if slow_index >= Max_sample_length
        Sample = Signal(:, 1 : Max_sample_length);       
        
        % Wait until calc finished
        if exist('func_result', 'var')
            fprintf("Calc Delayed.\n");
            wait(func_result);
        end
        % Detect vital signal in parallel
        func_result = parfeval(@Detect_Vital_parallel, 5, Sample, FPS);
        
        slow_index = 0;
    end
    
    %% Show Detected Vital Signal
    if exist('func_result', 'var') && strcmp(func_result.State, 'finished') % Calc is finished
        % Get result
        [complete_index, Selected_Signal, Selected_Index, Freq, Phase, error] = fetchNext(func_result);
        
        % Print result
        axh(2) = subplot(2,1,2);
        fprintf("%.1f s > ", (index - Min_slow_index)/FPS);
        if isempty(error)
            if isempty(Vital_Signal)
                Shifted_Signal = normalize(Selected_Signal);
                Prev_phase = Phase;
                Prev_freq = Freq;
            else
                Shifted_Signal = Phase_shift(normalize(Selected_Signal), Prev_freq, Prev_phase, Freq, Phase, FPS);
                Prev_phase = Phase;
                Prev_freq = Freq;
            end
            Vital_Signal = [Vital_Signal Shifted_Signal];
            
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
        clear func_result;
    end
end

radar.close();

%% Extract Vital Signal
if isempty(Vital_Signal)
    return
else
    showType = 'fft';
    switch showType
        case 'hht'
            figure(2)
            imf = emd(Selected_Signal, 'Display', 0);
            hht(imf, FPS, 'FrequencyResolution', 0.05, 'FrequencyLimits', [0 5] );
            
        case 'fft'
            % Extract Vital Signal by FFT
            [RR, HR, error] = Extract_Vital(Vital_Signal, FPS);
            
            % Print result
            if isempty(error)
                fprintf("Sample Length : %d bins / Spent time : %d ms /  Respiration Rate : %f bpm / Heart Rate : ", length(Vital_Signal), length(Vital_Signal)*1000/FPS, RR);
                fprintf("%f bpm ", HR);
                fprintf("\n");
            else
                fprintf(error);
            end
    end
    
end

