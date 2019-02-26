N = 5;
x = 100*rand(N,2);
% L = [ 2 -1 -1  0  0;...
%      -1  3 -1 -1  0;...
%      -1 -1  5 -1 -1;...
%       0 -1 -1  3 -1;...
%       0  0 -1 -1  2];
L = [ 2 -1  0  0 -1;...
     -1  2 -1  0  0;...
      0 -1  2 -1  0;...
      0  0 -1  2 -1;...
     -1  0  0 -1  2];
figure(1)
hold on
Lambda = eig(L);
lambda2 = Lambda(2);
dt = 0.1/lambda2;
for t = 0:dt:10/lambda2
    u = -L*x;
    % u(1,:) = 0;
    x = x+u*dt;
    scatter(x(:,1),x(:,2))
    % pause()
end