function pos = getPos(tau_vec, t, P)
ts = [0; cumsum(tau_vec)];
% tau = zeros(length(t),1);
pos = zeros(length(t),size(P,2));
for i=1:length(t)
    k = find(ts<t(i));
    k = k(end);
    tau = t(i)-ts(k);
    pos(i,:) = [tau^9,tau^8,tau^7,tau^6,tau^5,tau^4,tau^3,tau^2,tau,1] * P(10*k:-1:10*(k-1)+1,:);
end