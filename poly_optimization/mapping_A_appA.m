function A = mapping_A_appA(tau)

% r = 4th derivative
r = 4;
n = 9;
A0 = zeros(r,n+1);
Atau = zeros(r,n+1);

for i = 0:r
    for j = 0:n
        if j == i
            m = 0:i-1;
            A0(i+1,j+1) = prod(j-m);
        end
        if j >= i
            m = 0:i-1;
            Atau(i+1,j+1) = prod(j-m)*tau^(j-i);
        end
    end
end

A = [A0;Atau];
