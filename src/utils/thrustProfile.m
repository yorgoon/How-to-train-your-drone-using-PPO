X = -5:0.01:5;
f = tanh(X)*7.5 + 7.5;
f1 = tanh(X+1.5)*(thrust/2-1/2) + thrust/2+1/2;
f2 = tanh(X-1.5)*(4.5-(thrust/2-1/2)) + 4.5-thrust/2+1/2;
f1 = tanh(X+1.5)*(thrust/2) + thrust/2;
f2 = tanh(X-1.5)*(5-(thrust/2)) + 5-thrust/2;
plot(X,f)
hold on
grid on
yline(thrust)
plot(X,f1+f2)
hold off