%% This function calibrates phases of the successive vital signals. Algorithm that this function uses is to estimate how many samples should be shifted by phase difference.
function [Result] = Phase_shift(Next_Signal, Prev_freq, Prev_phase, Next_freq, Next_phase, FPS)
phase = round(Prev_phase * FPS / Prev_freq - Next_phase * FPS / Next_freq);	% phase parameter * sampling rate / frequency parameter(what we get is angular frequency) = the number of samples  
avg_freq = (Prev_freq + Next_freq) / 2;
remainder_sample = mod(phase, round(FPS/avg_freq));
	if remainder_sample == 0
	    Result = Next_Signal;
	else
	    Result = circshift(Next_Signal, remainder_sample);
	end
end
