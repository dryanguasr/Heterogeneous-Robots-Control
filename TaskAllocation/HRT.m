function TA = HRT( robot )
%HRT Hungarian Reallocate Tasks
%   Using a centralized algorithm determines the optimal task allocation
%   for the current stage of the mission
%% Memory Preallocation
Nr = size(robot,1);
TA = zeros(2,Nr);
Nv = size(robot{1}.VictimStatus,1);
Crt = zeros(Nv+Nr,Nr); % Capacity of the robot i for executing the task j
Tt = Inf*ones(Nv+Nr,Nr); % Time required for the robot i to complete the task j
%% Determines the cost asociated to each task (its completion time)
for r = 1:1:Nr
    for v = 1:1:Nv
        Crt(v,r) = ((robot{r}.Tc(3) == 1 && robot{r}.VictimStatus(v) == 3) || ...
                    (robot{r}.Tc(4) == 1 && robot{r}.VictimStatus(v) == 1) || ...
                    (robot{r}.Tc(5) == 1 && robot{r}.VictimStatus(v) == 0)   )...
                    && robot{r}.VictimID(v) == 1;
        if(Crt(v,r) == 1)
            % Evaluates the minimum distance to the victim and compares it
            % tho its speed
            Pv = robot{r}.VictimPos(v,:)';
            PVicR = [round(Pv(1)/robot{1}.Mres),...
                     round(Pv(2)/robot{1}.Mres)];
            Vr = robot{r}.V0;
            Tt(v,r) = robot{r}.Flood(PVicR(1),PVicR(2))/Vr;
            % if the task is evacuation considers the returning time
            if(robot{r}.Tc(5) == 1 && robot{r}.VictimStatus(v) == 0)
                Pr = robot{r}.HomePos';
                PRetR = [round(Pr(1)/robot{1}.Mres),...
                         round(Pr(2)/robot{1}.Mres)];
                Tt(v,r) = 2*Tt(v,r) + robot{r}.Flood(PRetR(1),PRetR(2))/Vr;
            end
        end
    end
end
%% Determines the estimated time of exploring for finding a victim
for r = 1:1:Nr
    for rn = Nv+1:1:Nv+Nr
        if(r==rn-Nv)
            Nmv = sum(robot{r}.VictimID == 0); % Number of Victims not found
            % Area Based
            Am = sum(sum(robot{r}.Map == 0))/robot{r}.Mres^2; % Area pending for exploration
            Vs = pi*robot{r}.Sr*robot{r}.V0; % estimation of the exploration speed [m^2/s]
            % Fm = sum(sum(robot{r}.dS))/robot{r}.Mres^2; % Frontier "Area" in cell
            Tt(rn,r) = Am/(Vs*Nmv);
            if(isnan(Tt(Nv+1,r)))
                Tt(rn,r) = Inf; % there is nothing else to explore
            end
        else
            Tt(rn,r) = Inf; % no robot can explore for other
        end
    end
end
%% Solve the allocation Problem
Art = munkres(Tt);
%% Changes the allocation's format to mission's format
for r = 1:1:Nr
    task = find(Art == r);
    if(isempty(task)) % there is no task allocated then it returns home
        TA(2,r) = 0;
        TA(1,r) = 2;
    elseif(task<=Nv)
        TA(2,r) = task;
        if(robot{r}.VictimStatus(TA(2,r)) == 0) % evacuate
            TA(1,r) = 3;
        elseif(robot{r}.VictimStatus(TA(2,r)) == 1) % deliver Kit
            TA(1,r) = 4;
        elseif(robot{r}.VictimStatus(TA(2,r)) == 3) % identify
            TA(1,r) = 4;
        end
    else % explore
        TA(2,r) = 0;
        TA(1,r) = 1;
    end
end
end