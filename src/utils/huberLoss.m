function loss = huberLoss(error, delta)
if error <= delta
    loss = 0.5*error^2;
else
    loss = delta*(abs(error) - 0.5*delta);
end

end