function [Selected_Signal, Selected_Index, error, info] = Detect_Vital_construct(Signal, FPS)
% Timer start
tStart = tic;

% Define
Sample_Index = length(Signal(1,:));
Sample_Length = length(Signal(:,1));
Rsquare_batch = zeros(1, Sample_Length);
error = [];

%% Processing received data
parfor fast_index = 1 : Sample_Length
    target = Signal(fast_index, :);
    slow_index = (0 : Sample_Index - 1);
    
    % Sinusoidal fitting
    [fitresult, gof] = fit(slow_index',target','sin1');
    coeff = coeffvalues(fitresult);
%     fit_signal = coeff(1)*sin(coeff(2)*slow_index+coeff(3));
    % Select Signal in range
    if (coeff(2)*FPS < 2*pi*0.5) && (coeff(2)*FPS > 2*pi*0.15) 
        Rsquare_batch(fast_index) = gof.rsquare;
    end
end

%% Post-processing processed data
% display Rsquare plot
% figure(2)
% subplot(2,1,1);
% stem(Rsquare_batch);

% Applying Moving Average filter & thresholding (abandon low Rsquare)
Rsquare_batch_filtered = zeros(1,length(Rsquare_batch));
window_size_half = 6; % actual size would be window_size_half * 2
Rsquare_threshold = 0.3; % value below 0.3 would be abandoned

parfor index = 1 : 1 : length(Rsquare_batch)
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

% subplot(2,1,2);
% plot(Rsquare_batch_filtered);

%% Find best Rsquare signal
best_fit_matrix_size = 6; % size of best matrix
best_fit_matrix = zeros(best_fit_matrix_size, Sample_Index);

[best_r_square,best_r_index] = maxk(Rsquare_batch_filtered, best_fit_matrix_size);

if min(best_r_square) < Rsquare_threshold
    Selected_Signal = [];
    Selected_Index = 0;
    error = "cannot detect vital signal.\n";
    return
end

% figure(3)
parfor i = 1 : length(best_r_index)
%     normalized_signal = normalize(Signal(best_r_index(i),1:Sample_Index));
%     best_fit_matrix(i,:) = detrend(Signal(best_r_index(i),1:Sample_Index)')';
    best_fit_matrix(i,:) = Signal(best_r_index(i),1:Sample_Index);
%     subplot(best_fit_matrix_size/3,3,i);
%     plot(best_fit_matrix(i,:));
%     xlabel("Best fit Signal : " + num2str(i) + newline + "Fast Index : " + num2str(best_r_index(i)) + newline + "Filtered R-square : " + num2str(Rsquare_batch_filtered(best_r_index(i))));
end

%% Construct Vital Signal
% applying low pass filter
lowpass_signal = lowpass(best_fit_matrix', 1, FPS);
best_fit_matrix_filtered = lowpass_signal';

% find zero crossing
figure(5)
zero_crossing = ones(best_fit_matrix_size, int32(Sample_Index/FPS));
for fit_index = 1 : best_fit_matrix_size
    target = best_fit_matrix_filtered(fit_index, :);
    num_zero = 1;
    for slow_index = 1 : (Sample_Index - 1)
        if target(slow_index) * target(slow_index+1) < 0
            if num_zero == 1
                zero_crossing(fit_index, num_zero) = slow_index;
                num_zero = num_zero + 1;
            elseif slow_index - zero_crossing(fit_index, num_zero - 1) > FPS
                zero_crossing(fit_index, num_zero) = slow_index;
                num_zero = num_zero + 1;
            end
        end
    end
    subplot(best_fit_matrix_size/3,3,fit_index);
    plot(target(:));
    hold on
    xlabel("Best fit Signal : " + num2str(fit_index) + newline + "Fast Index : " + num2str(best_r_index(fit_index)) + newline + "Filtered R-square : " + num2str(Rsquare_batch_filtered(best_r_index(fit_index))));
    plot(zero_crossing(fit_index,:), target(zero_crossing(fit_index,:)), 'o');
    hold off
end

% construct vital signal
corr_width = zeros(1,best_fit_matrix_size);
constructed_vital_signal = [];
constructed_signal_index = [];
for zero_index = 1 : length(zero_crossing(1,:))
    if zero_index == 1
        [max_zero_crossing, fit_index] = max(zero_crossing(:,zero_index));
        selected_sub_Signal = best_fit_matrix(fit_index, 1 : max_zero_crossing);
    else
        for fit_index = 1 : best_fit_matrix_size
            sub_Signal = best_fit_matrix(fit_index, zero_crossing(fit_index,zero_index - 1) : zero_crossing(fit_index,zero_index));
            
            if mean(prev_Signal) * mean(sub_Signal) >= 0
                sub_Signal = -sub_Signal;
            end
            
            temp_Signal = [constructed_vital_signal sub_Signal];
            [correlation, lags] = xcorr(temp_Signal,temp_Signal);
            
            for j = 1:length(lags) % length(lags) = (length(apppended_constructed_vital_signal)-1)*2 + 1
                if lags(j) == 0
                    lags_zero_index = j;
                end
            end
            correlation = correlation / correlation(lags_zero_index);
            
            r_3db = 1/sqrt(2);
            for j = 0:length(temp_Signal)-1
                if correlation(lags_zero_index + j) < r_3db
                    lags_3db = lags_zero_index + j;
                    break;
                end
            end
            corr_width(fit_index) = (lags_3db - lags_zero_index) * 2;
        end
        
        if max(corr_width) ~= 0
            [~, max_corr_index] = max(corr_width);
            selected_sub_Signal = best_fit_matrix(max_corr_index, zero_crossing(max_corr_index,zero_index - 1) : zero_crossing(max_corr_index,zero_index));
            if mean(prev_Signal) * mean(selected_sub_Signal) >= 0
                selected_sub_Signal = -selected_sub_Signal;
            end
            constructed_signal_index = [constructed_signal_index best_r_index(max_corr_index)];
        end
    end
    constructed_vital_signal = [constructed_vital_signal selected_sub_Signal];
    prev_Signal = selected_sub_Signal;
end

% Return signal, index
Selected_Signal = constructed_vital_signal;
Selected_Index = mode(constructed_signal_index);

% Timer stop
tSpend = toc(tStart);

% Return others(Rsquare, Spend time)
info.Rsquare = Rsquare_batch_filtered;
info.Time = tSpend;
end