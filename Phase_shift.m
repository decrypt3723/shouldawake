function [Result] = Phase_shift(Next_Signal, Prev_freq, Prev_phase, Next_freq, Next_phase, FPS)
phase = round(Prev_phase * FPS / Prev_freq - Next_phase * FPS / Next_freq);
avg_freq = (Prev_freq + Next_freq) / 2;
if mod(phase, round(FPS/avg_freq)) == 0
    Result = Next_Signal;
else
    Result = circshift(Next_Signal, phase);
end
end