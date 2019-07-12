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

%% Settings

% Input parameters
COM = 'COM6';
FPS = 110;
dataType = 'rf';

% Chip settings
PPS = 13;
DACmin = 949;
DACmax = 1100;
Iterations = 64;
FrameStart = 0.3; % meters.
FrameStop = 2.0; % meters.
FrameOffset = 0.18; % meters

% Load the library
Lib = ModuleConnector.Library;
Lib.libfunctions

% Loopback filter
alpha = 0.97;

% Search Setting
Maxlength = 1567;
MaxIndex = 1000;



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
radar.radarInstance.x4driver_set_frame_area_offset(FrameOffset);
radar.radarInstance.x4driver_set_frame_area(FrameStart,FrameStop);
% Read back actual set frame area
[frameStart, frameStop] = radar.radarInstance.x4driver_get_frame_area();

%% Start streaming and subscribe to message_data_float.

radar.start();

slow_index = 0;
binLength = 0;


% disp figure
f1 = figure(1);
clf(f1);

%% Receive Signal from Radar
while ishandle(1)
    % Peek message data float
    numPackets = radar.bufferSize();
    if numPackets > 0
        slow_index = slow_index+1;
        
        [frame, count] = radar.GetFrameNormalized(); % Get frame (uses read_message_data_float)
        if slow_index == 1
            % disp graph
            ph = plot(0);
            ylabel('Normalized amplitude');
            xlabel('Range [m]');
            
            th = title('');
            grid on;
            
            % Fast time Index
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
            Signal = zeros(numBins, MaxIndex-1);
        end
        
        %% Pre-Processing(loopback filter)
        switch dataType
            case 'rf'
                % Loopback filter
                Clutter = alpha*Clutter+(1-alpha)*frame;
                RawData =  frame - Clutter;
                
                Signal(:,slow_index) = RawData;
                ph.YData = RawData;
                ylim([-0.01 0.01]);
            case 'bb'
                frame = frame(1:end/2) + 1i*frame(end/2 + 1:end);
                
                % Loopback filter
                Clutter = alpha*Clutter+(1-alpha)*frame;
                RawData =  frame - Clutter;
                
                Signal(:,slow_index) = abs(RawData);
                ph.YData = abs(RawData);
                ylim([-0.01 0.01]);
        end
        
        th.String = ['FrameNo: ' num2str(slow_index) ' / Length: ' num2str(length(frame))];
        
        drawnow;
    end
    if slow_index >= MaxIndex
        break;
    end
end

%% signal ranging parameters
min_fast_index = 30;
min_slow_index = 200;
slow_sample_length = 256;
Rsquare_batch = zeros(1, length(Signal(:,1)));


%% Processing received data
for fi = min_fast_index : 1 : length(Signal(:,1))
    % sinusoidal fitting 
    one_fast_index_signal = Signal(fi, min_slow_index : min_slow_index + slow_sample_length - 1 );
    fast_index = (0 : slow_sample_length - 1);
    [fitresult, gof] = fit(fast_index',one_fast_index_signal','sin1');
    coeff = coeffvalues(fitresult);
    fit_signal = coeff(1)*sin(coeff(2)*fast_index+coeff(3));
    Rsquare_batch(1,fi) = gof.rsquare;
    
    % plot fitted signal and original signal together
    if (coeff(2)*FPS < 2*pi*0.5) && (coeff(2)*FPS > 2*pi*0.15) && (gof.rsquare > 0.3)
        plot(fast_index, fit_signal);
        hold on
        plot(fast_index, one_fast_index_signal);
        xlabel( num2str ( coeff(2)*FPS / (2*pi) + " Hz / " + "fast index : " + num2str(fi) + "// Rsquare : " + num2str(gof.rsquare) ) );
        ylabel("");
        hold off
        saveas(f1, ['result_' num2str(fi) '.jpg']);
        pause(0.2);
    end
end

%% Post-processing processed data 

    % display Rsquare plot
    figure
    subplot(2,1,1);
    stem(Rsquare_batch);
    
    % Applying Moving Average filter & thresholding (abandon low Rsquare)
    Rsquare_batch_filtered = zeros(1,length(Rsquare_batch));
    window_size_half = 20; % actual size would be window_size_half * 2
    Rsquare_threshold = 0.3; % value below 0.3 would be abandoned
    
    for index = 1 : 1 : length(Rsquare_batch)
        sum = 0;
        for window_index = (index - window_size_half) : 1 : (index + window_size_half)
            if (window_index < 1)
                temp = 0;
            elseif (window_index > length(Rsquare_batch))
                temp = 0;    
            elseif (Rsquare_batch(1 , window_index) < Rsquare_threshold)
                temp = 0;
            else
                temp = Rsquare_batch(1 , window_index);
            end
                sum = sum + temp;
        end
        Rsquare_batch_filtered(1 , index) = sum / (window_size_half * 2 + 1);
    end
    subplot(2,1,2);
    plot(Rsquare_batch_filtered);
    
    
%% Construct Vital Sign
    % ! CAUTION ! : I will remove first 200 samples (slow index) from
    % Signal Matrix.
    Signal = Signal(:,min_slow_index:end);
    % Best R-square Signal Matrix
    best_fit_matrix_size = 9; % size of best matrix
    best_fit_matrix = zeros(best_fit_matrix_size, slow_sample_length);
    zero_crossings = zeros(best_fit_matrix_size);
    [~,index] = maxk(Rsquare_batch_filtered, best_fit_matrix_size);
    % Find best Rsquare signal
    figure
    for i = 1 : length(index)
        best_fit_matrix(i,:) = Signal(index(i),1:slow_sample_length);
        subplot(3,3,i);
        plot(best_fit_matrix(i,:));
        xlabel("Best fit Signal : " + num2str(i) + newline + "Fast Index : " + num2str(index(i)) + newline + "Filtered R-square : " + num2str(Rsquare_batch_filtered(index(i))));
    end
    % applying kalman filter
    figure
    for i = 1 : length(index)
        dt = 1/FPS;  % sampleTime
        A = [1 dt; 0 1];
        H = [1 0];
        Q = [1 0; 0 10];
        R = 100;
        x = [0 3]';
        P = 3*eye(2);
        tmpPos = zeros(1,slow_sample_length);
        tmpVel = zeros(1,slow_sample_length);
        for k = 1:slow_sample_length    
            x_hat = A*x;
            P_hat = A*P*A' + Q;

            K = P_hat*H'/(H*P_hat*H' + R);

            x = x_hat + K*((10000*best_fit_matrix(i,k)) - H*x_hat);
            P = P_hat - K*H*P_hat;

            tmpPos(k) = x(1);
            tmpVel(k) = x(2);
        end
        subplot(3,3,i);
        plot(tmpVel);
    end
    % Find zero_crossings
    %
    %
radar.close();


