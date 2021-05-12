function [InitialObservation, LoggedSignal] = myResetFunction()
% Reset function to place custom quadrotor environment into a random
% initial state.
s0 = zeros(16,1);
s0(1:3) = s0(1:3) + 0.1*rand(3,1);
s0(7) = pi/2;
LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;
end

