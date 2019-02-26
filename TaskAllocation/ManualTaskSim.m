%% Simulation Parameters
dt = .125; % [s]
Tf = 30; % [s]
%% Scene Definition
MW = 40;%m
MH = 40;%m
Mres = 1; % [m/celda]
% creates the scene 
scene = ones(MW/Mres,MH/Mres);
% External Boarders
for i = 1:1:size(scene,1)
    scene(i,1) = 2;
    scene(i,size(scene,2)) = 2;
end
for j = 1:1:size(scene,2)
    scene(1,j) = 2;
    scene(size(scene,1),j) = 2;
end
% Internal Walls
for i = 1:1:round(.8*size(scene,1))
    for j = round(0.475*MH/Mres):1:round(0.525*MH/Mres)
        scene(i,j) = 2;
    end
end
for i = round(0.475*MW/Mres):1:round(0.525*MW/Mres)
    for j = 1:1:.2*size(scene,1)
        scene(i,j) = 2;
    end
end
for i = round(0.475*MW/Mres):1:round(0.525*MW/Mres)
    for j = round(.7*size(scene,1)):1:size(scene,1)
        scene(i,j) = 2;
    end
end
% obstacles randomly allocated
% Nobs = 10;
% Pobs = [0;0];
% for i = 1:1:Nobs
%     Pobs(1) = randi([2,size(scene,1)-1]);
%     Pobs(2) = randi([2,size(scene,2)-1]);
%     for x = -1:1:1
%         for y = -1:1:1
%             scene(Pobs(1)+x,Pobs(2)+y) = 2;
%         end
%     end
% end
%% defines victims randomly allocated (3 = victim non detected)
Nv = 7;
victim = cell(Nv,1);
Sr = 20;
Cr = 2;
% Type = 1*ones(1,Nv); % Tipo de la víctima (0->movil, 1->inmovil, 2->falsa,
                       % 3->desconocido,4->atendida)
Type = randi([0,2],1,Nv); % Tipo de la víctima (0->movil, 1->inmovil, 2->falsa,
                          % 3->desconocido,4->atendida)
V0 = 10 * (Type == 0);
Kar = 20;
for i=1:1:Nv
    pos = [randi([1,MW/Mres]);randi([0.5*MH/Mres,MH/Mres])];
    % reallocates the victims placed into non accesible areas
    while(scene(pos(1)-1,pos(2)-1)==2 || scene(pos(1)  ,pos(2)-1)==2 || scene(pos(1)+1,pos(2)-1)==2 || ... 
          scene(pos(1)-1,pos(2)  )==2 || scene(pos(1)  ,pos(2)  )==2 || scene(pos(1)+1,pos(2)  )==2 || ... 
          scene(pos(1)-1,pos(2)+1)==2 || scene(pos(1)  ,pos(2)+1)==2 || scene(pos(1)+1,pos(2)+1)==2 )
        pos = [randi(MW/Mres);randi([0.5*MH/Mres,MH/Mres])];
    end 
    victim{i} = Victim(i, Type(i), pos*Mres, V0(i), Sr, Cr, Kar);
    %scene(pos(1),pos(2)) = 3;
end
h = figure(1);
DScene = displayMap(scene);
image([0 MW], [0 MH], DScene,'CDataMapping','scaled')
hold on
for vic=1:1:Nv
    scatter(victim{vic}.Pos(2),victim{vic}.Pos(1),'filled','db')
end
saveas(h,'Mapa.jpg')
saveas(h,'Mapa')
hold off
%% Robots definition
% defines the tester robots
Nr = 5;
robot = cell(Nr,1);
Cr = 30; % entre 30 y 45 m de alcance para Wi-Fi - Dato tomado de 
         % http://pyme.lavoztx.com/
         % qu-distancia-todava-funciona-un-router-inalmbrico-7195.html
% V0a = 8*ones(Nr,1); % Homogeneous
V0a = 1*[9 8 7 6 5 4 3]; % Heterogeneous
Tca = [1 1 1 1 1;... % Task capabilities
       1 1 1 1 1;... % column 1 explore and Map
       1 1 1 1 1;... % column 2 Identify posible Victim (and mark)
       1 1 1 1 1;... % column 3 determine Victim's Type
       1 1 1 1 1];   % column 4 carry First Aid Kit(s)
                     % column 5 victim evacuation
for i=1:1:Nr
    V0 = V0a(i);
    Sr = 9;
    Kp = V0;
    pos = [2*(i)+4 ;3];
    Tc = Tca(i,:);
    robot{i} = Robot(i, MW, MH, Mres, pos, V0, Sr, Cr, Kp, Tc);
    robot{i}.TaskList = [1 2 3 4]; % everyone explores and then returns home
end
% Hand Defined Adjacency for line config.
A = [0 1 0 0 0;...
     1 0 1 0 0;...
     0 1 0 1 0;...
     0 0 1 0 1;...
     0 0 0 1 0];
% Hand Defined Adjacency for full config.
% A = [0 1 1 1 1;...
%      1 0 1 1 1;...
%      1 1 0 1 1;...
%      1 1 1 0 1;...
%      1 1 1 1 0];
D = diag(sum(A,2));
Laplacian = D-A;
Nn = [Laplacian(1,1) Laplacian(2,2) Laplacian(3,3)...
      Laplacian(4,4) Laplacian(5,5)];
for i=1:1:Nr
    robot{i}.Laplacian = Laplacian;
    robot{i}.Nn = Nn(i);
    robot{i} = robot{i}.updateNeighbors();
    robot{i}.VictimID = zeros(Nv,1);
    robot{i}.VictimPos = zeros(Nv,2);
    robot{i}.VictimStatus = zeros(Nv,1);
end
%% Time evolution
PosRobHist = zeros(2,Nr,round(Tf/0.5)+1);
PosVicHist = zeros(2,Nv,round(Tf/0.5)+1);
for i=1:1:Nr
    PosRobHist(:,i,1) = robot{i}.Pos;
end
for i=1:1:Nv
    PosVicHist(:,i,1) = victim{i}.Pos;
end
% Create Video writers
MinTimeVideo = VideoWriter('MinTime.avi');
MapVideo = VideoWriter('Map.avi');
%FrontierVideo = VideoWriter('Frontier.avi');
VoronoiCenterVideo = VideoWriter('VoronoiCenter.avi');
% Edit Frame Rates
MinTimeVideo.FrameRate = 10;
MapVideo.FrameRate = 10;
VoronoiCenterVideo.FrameRate = 10;
% open Video Writers
open(MinTimeVideo);
open(MapVideo);
%open(FrontierVideo);
open(VoronoiCenterVideo);
for t = 0:dt:Tf
    disp(['tiempo = ' num2str(t) ' segundos - ' num2str(100*t/Tf) '% Completado'])
    %% Robot's Potentials cleaning
    for r=1:1:Nr
        robot{r} = robot{r}.cleanRobotPotential();
    end
    %% Distance Calculus
    for r = 1:1:Nr
        % robot{r} = robot{r}.flood();
        robot{r} = robot{r}.euclideanFlood(); % cost warning also note there is an error of 8% at 22.5°
        %robot{r} = robot{r}.eucDis(); 
    end
    %% Map Updating (non optimized)
    % individual robot scene sensing
    for r = 1:1:Nr %for each robor
        % Line Of Sight Calculus
        LoS = lineOfSight(scene, Mres,robot{r}.Pos);
        % LoS = ones(size(scene));
        robot{r}.LoS = LoS;
        if(robot{r}.Tc == 1) % the robot can map
            for i = 1:1:MW/Mres %for each x pos (can be reduced to the sensed area)
                for j = 1:1:MH/Mres % for each y pos (can be reduced to the sensed area)
                    Drp = sqrt((robot{r}.Pos(1)-(i-1)*Mres)^2+...
                        (robot{r}.Pos(2)-(j-1)*Mres)^2); % Euclidean distance
                    if(Drp < robot{r}.Sr && LoS(i,j) == 1)
                        robot{r}.Map(i,j) = scene(i,j);
                    end
                    % the non accesible areas are set to -1
                    if(robot{r}.Flood(i,j) == MW/Mres * MH/Mres)
                        robot{r}.Map(i,j) = scene(i,j);
                    end
                end
            end
        end
    end
    % Map broadcasting (non optimized)
    for i = 1:1:Nr %for each robor
        for j = 1:1:Nr %for each robor
            if(i~=j)
                robot{i} = robot{i}.updateMap(robot{j}.Map);
            end
        end
    end
    %% Victims Behavior
    for i=1:1:Nv
        victim{i} = victim{i}.cleanVictimPotential();
        % avoids victim-victim collitions
%         for j=1:1:Nv
%             if(i~=j)
%                 victim{i} = victim{i}.conectToVictim(victim{j});
%             end
%         end
        % avoids victim-robot collitions and sets follow signals if
        % necessary
        for j=1:1:Nr
            Drv = sqrt((robot{j}.Pos(1)-victim{i}.Pos(1))^2+...
                       (robot{j}.Pos(2)-victim{i}.Pos(2))^2);
            PVicR = [round(victim{i}.Pos(1)/Mres),...
                     round(victim{i}.Pos(2)/Mres)];
            if(Drv < victim{i}.Sr  && robot{j}.LoS(PVicR(1),PVicR(2)) == 1)
                victim{i} = victim{i}.conectToRobot(robot{j});
                % broadcast the status information
                for k = 1:1:Nr
                    if(robot{k}.VictimStatus(i) ~= 4)
                        robot{k}.VictimStatus(i) = robot{j}.VictimStatus(i);
                    end
                end
            end       
            if(Drv < robot{j}.Sr && robot{j}.LoS(PVicR(1),PVicR(2)) == 1)
                % last know position updated and broadcasted
                for k = 1:1:Nr
                    robot{k}.VictimPos(i,:) = victim{i}.Pos;
                end
                if(robot{j}.VictimID(i) == 0) % 1st time detected)
                    % task reallocation
                    in = input(['el robot ' num2str(j) ' detectó a la víctima ' num2str(i)...
                                ' de tipo ' num2str(victim{i}.Type) ' nuevas tareas?\n']);
                    while(size(in,1) ~= 2 || size(in,2) ~= Nr)
                        in = input('asignación de tareas inválida, intente nuevamente \n');
                    end
                    % al the robots are notified
                    for k = 1:1:Nr
                        robot{k}.VictimID(i) = 1;
                        robot{k}.VictimTar = in(2,k);
                        robot{k}.CurrentTask = in(1,k);
                    end
                end
                % collition avoiding and tracking if needed
                robot{j} = robot{j}.conectToVictim(victim{i});
            end
        end
        victim{i} = victim{i}.detectObstacle(scene,MW,MH,Mres);
        [victim{i},u] = victim{i}.controlSignal;
        victim{i} = victim{i}.move(u,dt);
    end
    %% Tasks Allocation and Execution
    HDiscoverage = 0;
    VoronoiCentroids = zeros(2,Nr);
    for r = 1:1:Nr
    % Events Identification
        % Check for end of exploration
        if(all(all(robot{r}.Map>0)==1) && robot{r}.Tc(1) == 1)
            if(r == 1 && robot{r}.Tc(1) == 1)
                % task reallocation
                in = input('los robots terminaron de explorar ^_^ nuevas tareas?\n');
                while(size(in,1) ~= 2 || size(in,2) ~= Nr)
                    in = input('asignación de tareas inválida, intente nuevamente \n');
                end
                % al the robots are notified
                for k = 1:1:Nr
                    robot{k}.VictimTar = in(2,k);
                    robot{k}.CurrentTask = in(1,k);
                end
            end
            robot{r}.Tc(1) = 0; % there is nothing to be explored
        end
    % Task Execution
    %% Exploración
        if (robot{r}.TaskList(robot{r}.CurrentTask) == 1) % Exploration    
            robot{r} = robot{r}.initializeVoronoi();
            for rn = 1:1:Nr
                if(r~=rn)
                    robot{r} = robot{r}.updateVoronoi(robot{rn});
                end
            end
            % Voronoi Cells Centroid Copmutation
            robot{r} = robot{r}.updateVc();
            if(robot{r}.Nvf ~= 0) % sets the robot to follow a neighbor 
                                  % if there is no visible frontier
                rn = robot{r}.NeighborsList(1);
                robot{r}.Vc = robot{rn}.Pos;
                robot{r}.Nvf = robot{rn}.Nvf +1;
                for i = 2:1:robot{r}.Nn
                    rn = robot{r}.NeighborsList(i);
                    if(robot{rn}.Nvf+1 < robot{r}.Nvf)
                        robot{r}.Vc = robot{rn}.Pos;
                        robot{r}.Nvf = robot{r}.Nvf+1;
                    end
                end
            end
            robot{r} = robot{r}.visibleVc();
            VoronoiCentroids(:,r) = robot{r}.Vc;
        end
        %% Home returning
        HomePos = [4 6; 8 11;9 6;12 11;14 6]';
        if (robot{r}.TaskList(robot{r}.CurrentTask) == 2) % Regreso a casa
            robot{r}.Vc = robot{r}.goTo(HomePos(:,r));
            VoronoiCentroids(:,r) = HomePos(:,r);
            dBuff = norm(robot{r}.Pos-HomePos(:,r));
%             if(dBuff<1)
%                 disp(['home alcanzado por el robot ' num2str(r)])
%             end
        end
        %% Evacuación
        if(robot{r}.TaskList(robot{r}.CurrentTask) == 3)
            Vic = victim{robot{r}.VictimTar};
            dBuff = norm(robot{r}.Pos-Vic.Pos);
            if(dBuff > 6*Mres)
                robot{r}.Vc = robot{r}.goTo(Vic.Pos);
                VoronoiCentroids(:,r) = Vic.Pos;
            else
                robot{r}.Vc = robot{r}.goTo(HomePos(:,r));
                VoronoiCentroids(:,r) = HomePos(:,r);
                PVicR = [round(Vic.Pos(1)/Mres),...
                         round(Vic.Pos(2)/Mres)];
                if(norm(robot{r}.Pos-HomePos(:,r)) < 2*Mres &&...
                        robot{j}.LoS(PVicR(1),PVicR(2)) == 1)
                    % task reallocation
                    in = input(['la víctima ' num2str(Vic.ID) ' fue evacuada por el robot '...
                                 num2str(r) ' nuevas tareas?\n']);
                    while(size(in,1) ~= 2 || size(in,2) ~= Nr)
                        in = input('asignación de tareas inválida, intente nuevamente \n');
                    end
                    % al the robots are notified
                    for k = 1:1:Nr
                        robot{k}.VictimStatus(Vic.ID) = 4; % Victim Attention Complete
                        robot{k}.VictimTar = in(2,k);
                        robot{k}.CurrentTask = in(1,k);
                    end
                end
            end
        end
        %% Entrega de suministros e identificación de Víctimas
        if (robot{r}.TaskList(robot{r}.CurrentTask) == 4 ||...
            robot{r}.TaskList(robot{r}.CurrentTask) == 5)
            Vic = victim{robot{r}.VictimTar};
            robot{r}.Vc = robot{r}.goTo(Vic.Pos);
            VoronoiCentroids(:,r) = Vic.Pos;
            dBuff = norm(robot{r}.Pos-Vic.Pos);
            PVicR = [round(Vic.Pos(1)/Mres),...
                     round(Vic.Pos(2)/Mres)];
            if(dBuff < 2*Mres &&...
               robot{j}.LoS(PVicR(1),PVicR(2)) == 1)
                % task reallocation
                    if(robot{r}.VictimStatus(Vic.ID) == 3)
                        in = input(['Víctima ' num2str(Vic.ID) ' de tipo ' num2str(Vic.Type)...
                                    ' identificada por el robot ' num2str(r)...
                                    ' nuevas tareas?\n']);
                    else
                        in = input(['Suministro entregado a la víctima ' num2str(Vic.ID) ...
                                    ' por el robot ' num2str(r) ' nuevas tareas?\n']);
                    end
                    while(size(in,1) ~= 2 || size(in,2) ~= Nr)
                        in = input('asignación de tareas inválida, intente nuevamente \n');
                    end
                    % all the robots are notified
                    for k = 1:1:Nr
                        robot{k}.VictimStatus(Vic.ID) = 4; % Victim Attention Complete
                        robot{k}.VictimTar = in(2,k);
                        robot{k}.CurrentTask = in(1,k);
                    end
            end
        end
    end
    %% Robots Movement
    for r = 1:1:Nr    
        % AR potentials approach for obstacles evasion and conectivity hold
        updateNetwork;
%         if((mod(t,10*dt)==0)||(robot{r}.RecNet == 1 && r>rn))
%             updateNetwork;
%         end
        for rn = 1:1:Nr
            if(r~=rn)
                robot{r} = robot{r}.conectToRobot(robot{rn});
            end
        end
        robot{r} = robot{r}.detectObstacle();
        % Control Signal
        [robot{r},u] = robot{r}.controlSignal(dt);
        robot{r} = robot{r}.move(u,dt);
        PosRobHist(:,r,round(t/dt)+2) = robot{r}.Pos;
    end
    for r=1:1:Nr
        robot{r}.Laplacian = Laplacian;
    end
    for i=1:1:Nv
        PosVicHist(:,i,round(t/dt)+2) = victim{i}.Pos;
    end
    %% Videos
    % Min Time Display
    image([0 MW], [0 MH], robot{1}.minTime,'CDataMapping','scaled');
    hold on
    for r=1:1:Nr
        scatter(robot{r}.Pos(2),robot{r}.Pos(1))
    end
    frame = getframe;
    writeVideo(MinTimeVideo,frame);
    hold off
%     %Fontier
%     image([0 MW], [0 MH], robot{3}.dS,'CDataMapping','scaled')
%     hold on
%     for r=1:1:Nr
%         scatter(robot{r}.Pos(2),robot{r}.Pos(1),'filled')
%     end
%     frame = getframe;
%     writeVideo(FrontierVideo,frame);
%     hold off
% Map Display
    DMap = displayMap(robot{1}.Map);
    image([0 MW], [0 MH], DMap,'CDataMapping','scaled')
    hold on
    for r=1:1:Nr
        scatter(robot{r}.Pos(2),robot{r}.Pos(1),'filled','r')
    end
    sColors = ['b','g','y'];
    for vic=1:1:Nv
        sColor = sColors(victim{vic}.Type+1);
        scatter(victim{vic}.Pos(2),victim{vic}.Pos(1),'filled',sColor)
    end
    frame = getframe;
    writeVideo(MapVideo,frame);
    hold off
% Voronoi Centers
    [DVor,Alpha] = displayVor(robot{3}.VAssign);
    im = image([0 MW], [0 MH], DVor,'CDataMapping','scaled');
    im.AlphaData = Alpha;
    hold on
    for r=1:1:Nr
        scatter(robot{r}.Pos(2),robot{r}.Pos(1),'filled','r')
        DVel = 0.25*robot{r}.Vel;
        quiver(robot{r}.Pos(2),robot{r}.Pos(1),DVel(2),DVel(1),'k')
    end
    for vic=1:1:Nv
        sColor = sColors(victim{vic}.Type+1);
        scatter(victim{vic}.Pos(2),victim{vic}.Pos(1),'filled',sColor)
        DVel = 0.25*victim{vic}.Vel;
        quiver(victim{vic}.Pos(2),victim{vic}.Pos(1),DVel(2),DVel(1),'b')
    end
    for r=1:1:Nr
        scatter(VoronoiCentroids(2,r),VoronoiCentroids(1,r))
    end
    % Links
    for r = 1:1:Nr
        for rn = 1:1:Nr
            if(r~=rn && r<rn && Laplacian(r,rn)==-1)
                if(norm(robot{r}.Pos-robot{rn}.Pos) <= 0.8 * robot{r}.Cr &&...
                   norm(robot{r}.Pos-robot{rn}.Pos) >= 2*Mres)
                    plot([robot{r}.Pos(2),robot{rn}.Pos(2)],...
                         [robot{r}.Pos(1),robot{rn}.Pos(1)],'b')
                else
                    plot([robot{r}.Pos(2),robot{rn}.Pos(2)],...
                         [robot{r}.Pos(1),robot{rn}.Pos(1)],'r')
                end
            end
        end
    end
    frame = getframe;
    writeVideo(VoronoiCenterVideo,frame);
    hold off
end
close(MinTimeVideo);
close(MapVideo);
% close(FrontierVideo);
close(VoronoiCenterVideo);
%% Robots Travel Plot
image([0 MW], [0 MH], scene,'CDataMapping','scaled')
hold on
for ID=1:1:Nr
    scatter(PosRobHist(2,ID,:),PosRobHist(1,ID,:),'filled')
end
for ID=1:1:Nv
    scatter(PosVicHist(2,ID,:),PosVicHist(1,ID,:))
end
hold off
saveas(h,'RobotTravel.jpg')
saveas(h,'RobotTravel')