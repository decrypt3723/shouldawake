function [Result] = Phase_shift(Next_Signal, Prev_freq, Prev_phase, Next_freq, Next_phase, FPS)
phase = int(Prev_phase * FPS / Prev_freq - Next_phase * FPS / Next_freq);
avg_freq = (Prev_freq + Next_freq) / 2;
if mod(phase, int(FPS/avg_freq)) == 0
    Result = Next_Signal;
elseif phase < 0 % Right Shift
    Result = [Next_Signal(1 : end + phase) Next_Signal(end + phase + 1 : end)];
else % Left Shift
    Result = [Next_Signal(phase + 1 : end) Next_Signal(1 : phase)];
end
end