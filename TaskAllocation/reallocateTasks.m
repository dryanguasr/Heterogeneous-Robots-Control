function TA = reallocateTasks( robot )
%reallocateTasks reallocates the robots tasks
%   Using a centralized algorithm determines the optimal task allocation
%   for the current stage of the mission
%% Memory Preallocation
Nr = size(robot,1);
TA = zeros(2,Nr);
VList = find((robot{1}.VictimStatus < 4 .* robot{1}.VictimID == 1) == 1);
Nt = size(VList,1); % Number of Victim related Tasks
Nv = size(robot{1}.VictimStatus,1);
MSize = max(Nr,Nt);
Crt = ones(MSize); % Capacity of the robot i for executing the task j
Tt = 9999*ones(MSize); % Time required for the robot i to complete the task j
%% Determines the cost asociated to each task (its completion time)
for r = 1:1:size(robot,1)
    for v = 1:1:Nt
        Crt(r,VList(v)) = ((robot{r}.Tc(3) == 1 && robot{r}.VictimStatus(VList(v)) == 3) || ...
                           (robot{r}.Tc(4) == 1 && robot{r}.VictimStatus(VList(v)) == 1) || ...
                           (robot{r}.Tc(5) == 1 && robot{r}.VictimStatus(VList(v)) == 0)   )...
                    && robot{r}.VictimID(VList(v)) == 1;
        if(Crt(r,VList(v)) == 1)
            % Evaluates the minimum distance to the victim and compares it
            % tho its speed
            Pt = robot{r}.VictimPos(v,:)';
            PVicR = [round(Pt(1)/robot{1}.Mres),...
                     round(Pt(2)/robot{1}.Mres)];
            Vr = robot{r}.V0;
            Tt(r,v) = robot{r}.Flood(PVicR(1),PVicR(2))/Vr;
            % if the task is evacuation considers the returning time
            if(robot{r}.Tc(5) == 1 && robot{r}.VictimStatus(VList(v)) == 0)
                Pr = robot{r}.HomePos';
                PRetR = [round(Pr(1)/robot{1}.Mres),...
                         round(Pr(2)/robot{1}.Mres)];
                Tt(r,v) = 2*Tt(r,v) + robot{r}.Flood(PRetR(1),PRetR(2))/Vr;
            end
        end
    end
end
%% Determines the Utility of exploring for each Robot if there are more robots that victims
if (nt<nv)
    for r = 1:1:size(robot,1)
        for v = Nt+1:1:Nv
            
        end
    end
end
%% Solve the allocation Problem
f = reshape(Tt,[1,MSize^2]);
intcon = 1:MSize^2;
A = zeros(1,MSize^2);
b = 99999999999999999;
Aeq = zeros(2*MSize,MSize^2);
% Each robot has only one task allocated
for i = 1:1:MSize
    for j = 1:1:MSize^2
        Aeq(i,j) = floor((j-1)/MSize)+1 == i;
    end
end
% Each task has only a robot allocated
for i = MSize+1:1:2*MSize
    for j = 1:1:MSize^2
        Aeq(i,j) = mod((j-1),MSize)+1 == i-MSize;
    end
end
beq = ones(2*MSize,1);
lb = zeros(1,MSize^2);
ub = ones(1,MSize^2);
options = optimoptions('intlinprog','Display','off');
[x, ~]= intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
Art = reshape(x,[MSize,MSize]); % Optimal robots allocation
%% Modify the output for TA format
for r = 1:1:size(robot,1)
    TA(2,r) = find(Art(r,:) == 1);
    if(robot{r}.VictimStatus(TA(2,r)) == 0) % evacuate
        TA(1,r) = 3;
    elseif(robot{r}.VictimStatus(TA(2,r)) == 1) % deliver Kit
        TA(1,r) = 4;
    elseif(robot{r}.VictimStatus(TA(2,r)) == 3) % identify
        TA(1,r) = 4;    
    elseif(robot{r}.Tc(1) == 1) % Explore
        TA(1,r) = 1;        
    else % return Home
        TA(1,r) = 2;        
    end
end
end