% Problem constants
Nr = 3; % Number of Robots
Pr = 40*rand(2,Nr); % Position of the robots (randomly allocated)
Vr = randi([1,20],1,Nr); % Robot's velocity (randomly assigned)
Nt = Nr; % Number of task (Initially equal to the robots number)
%Crt = rand(Nr,Nt);
Crt = rand(Nr,Nt); % Capacity of the robot i for executing the task j
Crt = Crt<.8; % determines the percentage of robots capable of making the tasks
Pt = 40*rand(2,Nt); % Position of the tasks (randomly allocated)
% Task allocation
Tt = zeros(Nr,Nt); % Time required for the robot i to reach the task j
for i = 1:1:Nr
    for j = 1:1:Nt
        if(Crt(i,j) == 1)
            Tt(i,j) = sqrt((Pt(1,j)-Pr(1,i))^2+(Pt(2,j)-Pr(2,i))^2)/Vr(i);
        else
            Tt(i,j) = 99999;
        end
    end
end
%% Mixed Integer Liner Programming solving
f = reshape(Tt,[1,Nr*Nt]);
intcon = 1:Nr*Nt;
A = zeros(1,Nr*Nt);
b = 9999;
Aeq = zeros(Nr+Nt,Nr*Nt);
% Each robot has only one task allocated
for i = 1:1:Nr
    for j = 1:1:Nr*Nt
        Aeq(i,j) = floor((j-1)/Nr)+1 == i;
    end
end
% Each task has only a robot allocated
for i = Nr+1:1:Nr+Nt
    for j = 1:1:Nr*Nt
        Aeq(i,j) = mod((j-1),Nr)+1 == i-Nr;
    end
end
beq = ones(Nr+Nt,1);
lb = zeros(1,Nr*Nt);
ub = ones(1,Nr*Nt);
[x, ~]= intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
Art = reshape(x,[Nr,Nt]); % Optimal robots allocation