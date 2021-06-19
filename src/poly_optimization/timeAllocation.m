function tau_vec = timeAllocation(PATH, gamma)

% initial guess
% Time intervals are chosen randomly between 1s ~ 3s
tau_vec = rand(1,size(PATH,1)-1)*2+1;

% epsilon for numerical gradient
epsilon_t = 1e-5;

disp('time allocation started...')
i = 1;
J_seq = [];
while 1
    J_grad =  timeAllocationCostGradient(tau_vec, PATH, gamma, epsilon_t);
    alpha = 1e-6;
    step = 2;
    J_test = [];
    for m=1:100
        tau_vec_test = tau_vec - J_grad/norm(J_grad)*alpha*step^(m-1);
        if find(tau_vec_test<0)
            disp('tau vector cannot have negative values.')
            break
        end
        traj_test = MinimumSnapTrajectory(tau_vec_test', PATH);
        J_test = [J_test timeAllocationCost(traj_test, gamma)];
        
        % If the latest cost is larger then previous one, escape the loop
        [~, minidx] = min(J_test);
        if minidx ~= length(J_test)
            break
        end
    end
    [minval, minidx] = min(J_test);
    % update tau
    tau_vec = tau_vec - J_grad/norm(J_grad)*alpha*step^(minidx-1);
    % If cost does not change more than 1% for 5 times, we assume it
    % reaches to the minimum.
    J_seq = [J_seq minval];
    i = i+1;
    if i == 10
        if ~any(abs(J_seq - J_seq(end))./J_seq(end) > .01)
            break
        end
        i = 1;
        J_seq = [];
    end
end
disp('time allocation done')

% update polynomial coefficients
P = MinimumSnapTrajectory(tau_vec', PATH).P;

% end
disp('Finished')

% Check trajectory
K = length(tau_vec);
ttt = [];
ttt = [ttt, linspace(0,tau_vec(1),50)];
for i=2:K
    ttt = [ttt, sum(tau_vec(1:i-1)) + linspace(0,tau_vec(i),50)];
end
X = [];
XX = [];
for i=1:length(tau_vec) % # of segments
    for ii = 1:size(PATH,2)
        X(:,ii) = polyval(P(10*i:-1:10*(i-1)+1,ii),linspace(0,tau_vec(i),50));
    end
    XX = [XX;X];
    if size(PATH,2) == 3
        plot3(X(:,1),X(:,2),X(:,3),'LineWidth',1.5)
        hold on
        grid on
        axis equal
    end
end
hold off
end