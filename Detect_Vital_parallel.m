function [Selected_Signal, Selected_Index, error] = Detect_Vital_parallel(Signal, FPS)
% Define
Sample_Index = length(Signal(1,:));
Sample_Length = length(Signal(:,1));
Rsquare_batch = zeros(1, Sample_Length);
Phase_list = zeros(Sample_Length);
Freq_list = zeros(Sample_Length);
error = [];

%% Pre-Processing received data
% Linear Trend Subtraction
% Sample = detrend(Sample')';

% Exponential Weighted filter
alpha = 0.45;
Signal = filter(alpha, [1 1-alpha], Signal')';

% Noise reduction in EMD
parfor fast_index = 1 : Sample_Length
    [imf, ~] = emd(Signal(fast_index, :),'Interpolation','pchip', 'Display', 0);    
    Signal(fast_index, :) = sum(imf, 2);
end

%% Processing received data
for fast_index = 1 : Sample_Length
    target = Signal(fast_index, :);
    slow_index = (0 : Sample_Index - 1);
    
    % Sinusoidal fitting
    [fitresult, gof] = fit(slow_index',target','sin1');
    coeff = coeffvalues(fitresult);
%     fit_signal = coeff(1)*sin(coeff(2)*slow_index+coeff(3));
    % Select Signal in range
    if (coeff(2)*FPS < 2*pi*0.5) && (coeff(2)*FPS > 2*pi*0.15) 
        Rsquare_batch(fast_index) = gof.rsquare;
        Freq_list(fast_index) = coeff(2)/(2*pi);
        Phase_list(fast_index) = coeff(3);
    end
end

%% Post-processing processed data
% display Rsquare plot
% figure(2)
% subplot(2,1,1);
% stem(Rsquare_batch);

% Applying Moving Average filter & thresholding (abandon low Rsquare)
Rsquare_batch_filtered = zeros(1,length(Rsquare_batch));
window_size_half = 6; % actual size would be window_size_half * 1
Rsquare_threshold = 0.3; % value below 0.3 would be abandoned
parfor index = 1 : 1 : length(Rsquare_batch)
    Rsquare_sum = 0;
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
        Rsquare_sum = Rsquare_sum + temp;
    end
    Rsquare_batch_filtered(1 , index) = Rsquare_sum / (window_size_half * 2 + 1);
end

% subplot(2,1,2);
% plot(Rsquare_batch_filtered);
% hold on

%% Select best fit signal
% Select Signal
[~, filtered_list] = maxk(max(abs(Signal')), 10);
filtered_range = min(filtered_list) : max(filtered_list);

% Select best r 
[r_value,best_r_index] = max(Rsquare_batch_filtered(filtered_range));
% plot(filtered_range(best_r_index), Rsquare_batch_filtered(filtered_range(best_r_index)), 'o');
% hold off
% drawnow;

Selected_Index = filtered_range(best_r_index);
Selected_Signal = Signal(Selected_Index, :);
Phase = Phase_list(Selected_Index);
Freq = Freq_list(Selected_Index)*FPS;

% Check Rsquare threshold
if r_value < Rsquare_threshold
    Selected_Index = 0;
    Selected_Signal = [];
    error = 'Cannot detect Vital Signal\n';    
end

end