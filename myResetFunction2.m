function [InitialObservation, LoggedSignal] = myResetFunction2()
% Reset function to place custom quadrotor environment into a random
% initial state.
s0 = zeros(12,1);
s0(1:3) = s0(1:3) + 0.1*rand(3,1);
s0(4:6) = s0(4:6) + 0.01*rand(3,1);
LoggedSignal.State = s0;
InitialObservation = LoggedSignal.State;
end

