function [RR, HR, error] = Extract_Vital(Selected_Signal, FPS)
% FFT
result = fft(Selected_Signal);
magnitude = abs(result(1:length(Selected_Signal)/2+1))*2;
freq = FPS * 60 *(1:length(Selected_Signal)/2 + 1)/length(Selected_Signal);

% Filter in range
freq_filtered = freq(freq < 300);
magnitude_filtered = magnitude(freq < 300);

figure(6)
title("FFT result");
plot(freq_filtered, magnitude_filtered);
hold on

% find respiration rate
startRange = find(freq >= 9, 1);
endRange = find(freq <= 30);

if isempty(endRange)
    error = 'Respiration rate cannot find.\n';
    return
end

[~, RR_index] = max(magnitude(startRange : endRange(end)));
RR = freq(RR_index - 1 + startRange);

plot(RR, magnitude_filtered(RR_index - 1 + startRange), 'ok');

% find peak in FFT result in 50 ~ 120 bpm
startRange = find(freq >= 50, 1);
endRange = find(freq <= 100);

if isempty(endRange)
    error = 'Heart rate cannot find.\n';
    return
end

[peak_vals, peak_index] = findpeaks(magnitude(startRange - 1 : endRange(end) + 1));
[~,max_peak_index] = maxk(peak_vals, 3);

Heart_rate = freq(peak_index(max_peak_index) - 1 + startRange - 1);

% Cancel harmonic
Heart_rate_filtered = freq(peak_index(max_peak_index(mod(Heart_rate, RR) ~= 0)) - 1 + startRange - 1);

plot(Heart_rate_filtered, magnitude_filtered(peak_index(max_peak_index(mod(Heart_rate, RR) ~= 0)) - 1 + startRange - 1), 'ok');
hold off

HR = Heart_rate_filtered;
error = [];

end