function Q = cost_mat_appA(tau)

r = 4;
Q = zeros(10);
QQ = zeros(10,10,length(r));
% eg optimizing 4th derivative
% r = 4;
% row
for rr = 1:length(r)
    for i=0:size(Q,1)-1
        % column
        for j=0:size(Q,2)-1
            if i >= r(rr) && j >= r(rr)
                m = 0:r(rr)-1;
                QQ(i+1,j+1,rr) = 2*prod((i-m).*(j-m))*tau^(i+j-2*r(rr)+1)/(i+j-2*r(rr)+1);
            end
        end
    end
end

for i = 1:length(r)
    Q = Q + QQ(:,:,i);
end