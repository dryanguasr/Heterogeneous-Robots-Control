%% Simulation Parameters
dt = .025; % [s]
Tf = 25; % [s]
RecSteps = 1; % cada cuantos pasos se actualiza la posición del centroide de la celda
%% Scene Definition
MW = 40;%m
MH = 40;%m
Mres = 1; % [m/celda]
% load the scene 
scene = ones(MW/Mres,MH/Mres);
load('EmpScene','scene'); % Empty Scene with boarders
% for i = 1:1:round(.8*size(scene,1))
%     for j = round(0.475*MH/Mres):1:round(0.525*MH/Mres)
%         scene(i,j) = 2;
%     end
% end
% for i = round(0.475*MW/Mres):1:round(0.525*MW/Mres)
%     for j = 1:1:.2*size(scene,1)
%         scene(i,j) = 2;
%     end
% end
% for i = round(0.475*MW/Mres):1:round(0.525*MW/Mres)
%     for j = round(.7*size(scene,1)):1:size(scene,1)
%         scene(i,j) = 2;
%     end
% end
% defines victims randomly allocated (3 = victim non detected)
Nv = 10;
victim = cell(Nv,1);
Sr = 10;
V0 = 7;
Kar = 1;
for i=1:1:Nv
    pos = [randi(MW/Mres);randi([0.5*MH/Mres,MH/Mres])];
    % reallocates the victims placed into non accesible areas
    while(scene(pos(1),pos(2))==2)
        pos = [randi(MW/Mres);randi([0.5*MH/Mres,MH/Mres])];
    end 
    victim{i} = Victim(i, pos*Mres, V0, Sr, Kar);
    %scene(pos(1),pos(2)) = 3;
end
%h=figure('Position',[100 100 580 500]);
h = figure(1);
DScene = displayMap(scene);
image([0 MW], [0 MH], DScene,'CDataMapping','scaled')
hold on
for vic=1:1:Nv
    scatter(victim{vic}.Pos(2),victim{vic}.Pos(1),'filled','yellow')
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
V0a = [10 9 8 7 6 5 4 3]; % Heterogeneous
% V0a = 8*ones(8,1); % Homogeneous
for i=1:1:Nr
    V0 = V0a(i);
    Sr = V0;
    Kp = V0;
    pos = [2*(i)+2 ;3];
    robot{i} = Robot(i, MW, MH, Mres, pos, V0, Sr, Cr, Kp);
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
% Laplacian = [4 1 1 1 1;... % Hand Defined Laplacian for full conection
%              1 4 1 1 1;...
%              1 1 4 1 1;...
%              1 1 1 4 1;...
%              1 1 1 1 4];
Nn = [Laplacian(1,1) Laplacian(2,2) Laplacian(3,3)...
      Laplacian(4,4) Laplacian(5,5)];
for i=1:1:Nr
    robot{i}.Laplacian = Laplacian;
    robot{i}.Nn = Nn(i);
    robot{i} = robot{i}.updateNeighbors();
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
% MinTimeVideo = VideoWriter('MinTime.avi');
MapVideo = VideoWriter('Map.avi');
%FrontierVideo = VideoWriter('Frontier.avi');
VoronoiCenterVideo = VideoWriter('VoronoiCenter.avi');
% open(MinTimeVideo);
open(MapVideo);
%open(FrontierVideo);
open(VoronoiCenterVideo);
for t = 0:dt:Tf
    disp(['tiempo = ' num2str(t) ' segundos - ' num2str(100*t/Tf) '% Completado'])
    %% Victims Behavior
    for i=1:1:Nv
        victim{i} = victim{i}.cleanVictimPotential();
%         for j=1:1:Nr
%             Drv = sqrt((robot{j}.Pos(1)-victim{i}.Pos(1))^2+...
%                        (robot{j}.Pos(2)-victim{i}.Pos(2))^2);
%             if(Drv < victim{i}.Sr && Drv < robot{j}.Sr)   
%                 victim{i} = victim{i}.conectToRobot(robot{j});
%                 robot{j} = robot{j}.conectToVictim(victim{i});
%             end
%         end
        u = victim{i}.controlSignal;
        victim{i} = victim{i}.move(u,dt);
    end
    %% Robot's Potentials cleaning
    for r=1:1:Nr
        robot{r} = robot{r}.cleanRobotPotential();
    end
    %% Distance Calculus
    for r = 1:1:Nr
        robot{r} = robot{r}.flood();
        % robot{r} = robot{r}.eucDis();
    end
    %% Map Updating (non optimized)
    % individual robot exploration
    for r = 1:1:Nr %for each robor
        % Line Of Sight Calculus
        LoS = lineOfSight(scene, Mres,robot{r}.Pos);
        robot{r}.LoS = LoS;
        % Los = ones(size(scene));
        for i = 1:1:MW/Mres %for each x pos (can be reduced to the sensed area)
            for j = 1:1:MH/Mres % for each y pos (can be reduced to the sensed area)
                %           Drp = abs(tester{r}.Pos(1)-(i-1)*Mres)/tester{r}.V0+...
                %                 abs(tester{r}.Pos(2)-(j-1)*Mres)/tester{r}.V0; % Manhattan distance
                Drp = sqrt((robot{r}.Pos(1)-(i-1)*Mres)^2+...
                    (robot{r}.Pos(2)-(j-1)*Mres)^2); % Euclidean distance
                if(Drp < robot{r}.Sr && LoS(i,j) == 1)
                    robot{r}.Map(i,j) = scene(i,j);
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
    %% Voronoi Exploration
    HDiscoverage = 0;
    VoronoiCentroids = zeros(2,Nr);
    if(mod(t,RecSteps*dt)==0)
        for r = 1:1:Nr
            % Voronoi Cells asignation
            robot{r} = robot{r}.initializeVoronoi();
            for rn = 1:1:Nr
                %if(robot{r}.Laplacian(r,rn) == 1 && r~=rn)
                if(r~=rn)
                    robot{r} = robot{r}.updateVoronoi(robot{rn});
                end
            end
            % Voronoi Phi & HDiscoverage Calculus
            robot{r} = robot{r}.updateHDis();
            HDiscoverage = HDiscoverage + robot{r}.HDis;
            % Voronoi Cells Centroid Copmutation
            robot{r} = robot{r}.updateVc();
            robot{r} = robot{r}.visibleVc();
            VoronoiCentroids(:,r) = robot{r}.Vc;
        end
    end
    %% Robots Movement
    for r = 1:1:Nr    
        % AR potentials approach for obstacles evasion and conectivity hold
        if((mod(t,10*dt)==0)||(robot{r}.RecNet == 1 && r>rn))
            updateNetwork;
        end
        for rn = 1:1:Nr
            if(r~=rn)
                robot{r} = robot{r}.conectToRobot(robot{rn});
            end
        end
        robot{r} = robot{r}.detectObstacle();
        % Control Signal
        [robot{r},u] = robot{r}.controlSignal(dt);
        robot{r} = robot{r}.move(u,dt);
        HDiscoverage = HDiscoverage + robot{r}.HDis;
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
%     image([0 MW], [0 MH], robot{1}.minTime,'CDataMapping','scaled');
%     hold on
%     for r=1:1:Nr
%         scatter(robot{r}.Pos(2),robot{r}.Pos(1))
%     end
%     frame = getframe;
%     writeVideo(MinTimeVideo,frame);
%     hold off
    % Fontier
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
        scatter(robot{r}.Pos(2),robot{r}.Pos(1),'filled')
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
        scatter(robot{r}.Pos(2),robot{r}.Pos(1),'filled')
        DVel = 0.25*robot{r}.Vel;
        quiver(robot{r}.Pos(2),robot{r}.Pos(1),DVel(2),DVel(1),'k')
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
% close(MinTimeVideo);
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
