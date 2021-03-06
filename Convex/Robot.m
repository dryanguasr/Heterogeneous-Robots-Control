classdef Robot
    properties
        % Caracter�sticas y estado del robot
        Pos = [0;0];         % Posici�n
        Vel = [0;0];         % Velocidad actual
        V0 = 1;              % Velocidad promedio [m/s]
        Sr = 1;              % Radio de Sensado [m]
        Cr = inf;            % Radio de Comunicaci�n [m]
        ID = 0;              % ID del robot en la misi�n
        % Caracter�sticas del mapa
        MW = 1;              % Ancho del mapa [m]
        MH = 1;              % Alto del mapa [m]
        Mres = 1;            % Resoluci�n del mapa [m]
        Flood = [];          % Distancia por inundaci�n
        % Exploraci�n del Mapa
        Map = [];            % Mapa propio de la escena
        maxVor = inf;        % M�ximo tiempo de llegada a un punto posible
        minTime = [];        % M�nimo tiempo de llegada a un punto
        LoS = [];            % Determina si la celda es visible o no
        VAssign = [];        % Asignaci�n de Celda de Voronoi
        dS = [];             % Frontera de exploraci�n
        dF = [];             % Distancia a la Frontera
        phi = [];            % Funci�n de potencial artificial
        HDis = 0;            % Funci�n de desempe�o de la exploraci�n
        Vc = [0;0];          % Centroide de la Celda de Voronoi Propia
        % Funci�nes de potencial de atracci�n y repulsi�n
        PotNet = [0;0];
        % Par�metros del Controlador
        Kp = 10;             % Constante proporcional para exploraci�n
        Kar = 1;             % Constante proporcional para atracci�n/repulsi�n
        PerSign = 0;         % Signo para la perturbaci�n
        PerDir = [0;0];      % Direcci�n de la perturbaci�n
        PerCon = 0;          % Contador para mantener Perturbaci�n
        % Lista de Tareas
        TaskList = [];       % Lista de Tareas
        % Conectivdad
        NeighborsList = [];  % Lista de Vecinos
        Nn = 0;              % N�mero de Vecinos
        Laplacian = [];      % Laplaciano del grafo
        RecNet = 0;          % Comando de reconfiguraci�n
    end
    methods
    % Constructor
        function r = Robot(ID, MW, MH, Mres, pos, V0, Sr, Cr, Kp)
            % Propiedades y estado del robot
            r.ID = ID;
            r.V0 = V0;
            r.Sr = Sr;
            r.Cr = Cr;
            r.Vc = pos;
            r.Kp = Kp;
            r.Pos = pos;
            r.Vel = zeros(2,1);
            % Mapa
            r.MW = MW;
            r.MH = MH;
            r.Mres = Mres;
            r.Map = zeros(MW/Mres,MH/Mres);
            r.Flood = MW/Mres*MH/Mres*ones(size(r.Map));
            % Exploraci�n
            r.maxVor = max(MW,MH)*sqrt(2);
            r.minTime = r.maxVor/r.V0 * ones(size(r.Map));
            r.VAssign = zeros(size(r.Map));
            r.dS = zeros(MW/Mres,MH/Mres);
            % Tareas
            r.TaskList = [];
        end 
    %% C�lculo de la distancia 
        % distancia por inundaci�n
        function r = flood(r)
            r.Flood = r.MW/r.Mres * r.MH/r.Mres * ones(size(r.Map));
            for i=1:1:size(r.Map,1)
                for j=1:1:size(r.Map,2)
                    if(r.Map(i,j) == 2)
                    % if(r.Map(i,j) ~= 1)    
                        r.Flood(i,j) = -1;
                    end
                end
            end
            Px = max(1,round((r.Pos(1)/r.Mres)));
            Py = max(1,round((r.Pos(2)/r.Mres)));
            r.Flood(Px,Py) = 1;
            dPile = zeros(size(r.Flood));
            cicle = 1;
            MaxCicles = r.V0/r.Mres;
            while(sum(sum(abs(r.Flood-dPile))) ~=0 && cicle < MaxCicles)
                dPile = r.Flood;
                cicle = cicle + 1;
                %up-down, left-right cicle
                for i=1:1:size(r.Map,1)
                    for j = 1:1:size(r.Map,2)
                        if(i-1>0)
                            if(r.Flood(i,j) > 0 && r.Flood(i-1,j) > r.Flood(i,j) && r.Flood(i-1,j) ~= -1)
                                r.Flood(i-1,j) = r.Flood(i,j) + 1;
                            end
                        end
                        if(i+1<=size(r.Flood,1))
                            if(r.Flood(i,j)>0&&r.Flood(i+1,j)>r.Flood(i,j)&&r.Flood(i+1,j)~=-1)
                                r.Flood(i+1,j) = r.Flood(i,j) + 1;
                            end
                        end
                        if(j-1>0)
                            if(r.Flood(i,j)>0&&r.Flood(i,j-1)>r.Flood(i,j)&&r.Flood(i,j-1)~=-1)
                                r.Flood(i,j-1) = r.Flood(i,j) + 1;
                            end
                        end
                        if(j+1<=size(r.Flood,2))
                            if(r.Flood(i,j)>0&&r.Flood(i,j+1)>r.Flood(i,j)&&r.Flood(i,j+1)~=-1)
                                r.Flood(i,j+1) = r.Flood(i,j) + 1;
                            end
                        end
                    end
                end
                %down-up, right-left cicle
                for i=size(r.Map,1):-1:1
                    for j = size(r.Map,2):-1:1
                        if(i-1>0)
                            if(r.Flood(i,j)>0&&r.Flood(i-1,j)>r.Flood(i,j)&&r.Flood(i-1,j)~=-1)
                                r.Flood(i-1,j) = r.Flood(i,j) + 1;
                            end
                        end
                        if(i+1<=size(r.Flood,1))
                            if(r.Flood(i,j)>0&&r.Flood(i+1,j)>r.Flood(i,j)&&r.Flood(i+1,j)~=-1)
                                r.Flood(i+1,j) = r.Flood(i,j) + 1;
                            end
                        end
                        if(j-1>0)
                            if(r.Flood(i,j)>0&&r.Flood(i,j-1)>r.Flood(i,j)&&r.Flood(i,j-1)~=-1)
                                r.Flood(i,j-1) = r.Flood(i,j) + 1;
                            end
                        end
                        if(j+1<=size(r.Flood,2))
                            if(r.Flood(i,j)>0&&r.Flood(i,j+1)>r.Flood(i,j)&&r.Flood(i,j+1)~=-1)
                                r.Flood(i,j+1) = r.Flood(i,j) + 1;
                            end
                        end
                    end
                end
            end
        end
        % distancia Euclidea
        function r = eucDis(r)
            r.Flood = zeros(size(r.Map));
            for i=1:1:size(r.Map,1)
                for j=1:1:size(r.Map,2)
                    r.Flood(i,j) = sqrt((r.Pos(1)-i*r.Mres)^2 + ...
                                        (r.Pos(2)-j*r.Mres)^2);
                end
            end
        end
    %% Funciones de Exploraci�n
        % Actualizaci�n del Mapa Basado en Mapas Recibidos
        function r = updateMap(r,Map)
            for i=1:1:size(Map,1)
                for j=1:1:size(Map,2)
                    if(Map(i,j) ~= 0)
                        r.Map(i,j) = Map(i,j);
                        if(Map(i,j)~=1)
                            r.Flood(i,j)=-1;
                        end
                    end
                end
            end
        end
        % Partici�n en celdas de Voronoi
        function r = initializeVoronoi(r)
            r.minTime = r.Flood/r.V0;
            r.VAssign = r.ID*ones(size(r.Map));
            for i=1:1:size(r.Map,1)
                for j=1:1:size(r.Map,2)
                    if(r.Map(i,j) == 2)
                        r.VAssign(i,j) = 0;
                        r.minTime(i,j) = 0;
                    end
                end
            end
        end
        function r = updateVoronoi(r,rn)
            for i = 1:1:r.MW/r.Mres %for each x pos
                for j = 1:1:r.MH/r.Mres % for each y pos
                    Trv = rn.Flood(i,j)/rn.V0; % Flooding Distance
                    if(Trv < r.minTime(i,j) && r.VAssign(i,j) ~= 0)
                        r.minTime(i,j) = Trv;
                        r.VAssign(i,j) = rn.ID;
                    end
                end
            end
        end
        % C�lculo de la funci�n de desempe�o de exploraci�n
        function r = updateHDis(r) 
            % Frontier definition (non optimized)
            r.HDis = 0;
            r.dS = zeros(size(r.Map));
            % Center of the map
            for i = 2:1:r.MW/r.Mres-1
                for j = 2:1:r.MH/r.Mres-1
                    if(r.VAssign(i,j) == r.ID)
                        if(r.Map(i,j) == 0 && ...
                                ((r.Map(i+1,j) ~= 0 && r.Map(i+1,j) ~= 2)||...
                                 (r.Map(i-1,j) ~= 0 && r.Map(i-1,j) ~= 2)||...
                                 (r.Map(i,j+1) ~= 0 && r.Map(i,j+1) ~= 2)||...
                                 (r.Map(i,j-1) ~= 0 && r.Map(i,j-1) ~= 2)))
                            r.dS(i,j) = 1;
                        end
                    end
                end
            end
            % Left Boarder
            for i = 1:1:r.MW/r.Mres
                if(r.VAssign(i,1) == r.ID)
                    if(r.Map(i,1) == 0 && r.Map(i,2) ~= 0)
                        r.dS(i,1) = 1;
                    end
                end
            end
            % Right Boarder
            for i = 1:1:r.MW/r.Mres
                if(r.VAssign(i,r.MH/r.Mres) == r.ID)
                    if(r.Map(i,r.MH/r.Mres) == 0 && r.Map(i,r.MH/r.Mres-1) ~= 0)
                        r.dS(i,r.MH/r.Mres) = 1;
                    end
                end
            end
            % Top Boarder
            for j = 1:1:r.MH/r.Mres
                if(r.VAssign(1,j) == r.ID)
                    if(r.Map(1,j) == 0 && r.Map(2,j) ~= 0)
                        r.dS(1,j) = 1;
                    end
                end
            end
            % Bottom Boarder
            for j = 1:1:r.MH/r.Mres
                if(r.VAssign(r.MW/r.Mres,j) == r.ID)
                    if(r.Map(r.MW/r.Mres,j) == 0 && r.Map(r.MW/r.Mres-1,j) ~= 0)
                        r.dS(r.MW/r.Mres,j) = 1;
                    end
                end
            end
            % distance to Frontier computing (Brute Force Computing)
            r.dF = zeros(r.MW/r.Mres,r.MH/r.Mres);
            for i = 1:1:r.MW/r.Mres % for each point in the VCell
                for j = 1:1:r.MH/r.Mres 
                    if ( r.VAssign(i,j) == r.ID && r.Map(i,j) == 1) 
                    % if ( r.VAssign(i,j) == r.ID && r.Map(i,j) ~= 0)     
                        for x = 1:1:r.MW/r.Mres %for each point in the Frontier
                            for y = 1:1:r.MH/r.Mres
                                if(r.dS(x,y)==1)
                                  % Drf = abs(i*r.Mres-x*r.Mres) +...
                                  %       abs(j*r.Mres-y*r.Mres); % Mannhattan
                                    Drf = sqrt((i*r.Mres-x*r.Mres)^2 +...
                                               (j*r.Mres-y*r.Mres)^2); % Euclidean
                                    % Drf = r.Flood(i,j);
                                    if(r.dF(i,j) > Drf || r.dF(i,j) == 0)
                                        r.dF(i,j) = Drf;
                                    end
                                end
                            end
                        end
                    end
                end
            end
            % phi function computing
            r.phi = zeros(r.MW/r.Mres,r.MH/r.Mres);
            for i = 1:1:r.MW/r.Mres %for each x pos (can be reduced to the sensed area)
                for j = 1:1:r.MH/r.Mres % for each y pos (can be reduced to the sensed area)
                    if(r.VAssign(i,j) == r.ID && r.Map(i,j) ~= 0)
                        r.phi(i,j) = r.V0*exp(-1/r.V0*r.dF(i,j)^2); 
                      % Drp = abs(i*r.Mres-r.Pos(1)) +...
                      %       abs(j*r.Mres-r.Pos(2)); % Euclidean
                        Drp = sqrt((i*r.Mres-r.Pos(1))^2 +...
                                   (j*r.Mres-r.Pos(2))^2); % Euclidean
                        r.HDis = r.HDis + Drp^2*r.phi(i,j)*r.Mres^2;
                    end
                end
            end
        end
        % C�lculo del centroide de la celda de Voronoi (por optimizar)
        function r = updateVc(r)
            r.Vc = [0;0];
            TotalMass = 0;
            for i = 1:1:r.MW/r.Mres % for each x pos (can be reduced to the
                                    % Voronoi Cell area)
                for j = 1:1:r.MH/r.Mres % for each y pos (can be reduced to
                                        % the Voronoi Cell area)
                    if(r.VAssign(i,j)==r.ID)
                        r.Vc = r.Vc + r.phi(i,j)*[(i*r.Mres);(j*r.Mres)];
                        TotalMass = TotalMass + r.phi(i,j);
                    end
                end
            end
            %disp(['robot ' num2str(r.ID) ' TM = ' num2str(TotalMass)])
            r.Vc = r.Vc./TotalMass;
            % breaks the equilibrium points
            if(norm(r.Vc-r.Pos) < r.Mres) 
                % disp(['centroide alcanzado por el robot ' num2str(r.ID)])
                % r.Vc = r.Vc + r.V0*rand()*(r.Vc-r.Pos)/norm(r.Vc-r.Pos);
                % r.Vc = r.Vc + r.V0*(r.Vc-r.Pos)/norm(r.Vc-r.Pos);
                % r.Vc = r.Vc - [r.V0;r.V0];
            end
%             
%             if(abs(r.Vc(1)) < 0.1 && abs(r.Vc(2)) < 0.001)
%                 r.Vc = r.Pos;
%             end
        end
        function r = visibleVc(r)
            it = 1;
            itMax = 250;
            ronda = 1;
            try
                Pvis = [round(r.Vc(1))/r.Mres, round(r.Vc(2))/r.Mres];
                while(r.LoS(Pvis(1),Pvis(2))~=1 &&...
                      r.Map(Pvis(1),Pvis(2))~=1 &&...
                      it<itMax)
                    it = it+1;
                    if(Pvis(1)+1 < size(r.Map,1))
                        if(r.Flood(Pvis(1)+1,Pvis(2)  )<...
                           r.Flood(Pvis(1)  ,Pvis(2)  )&&...
                           r.Flood(Pvis(1)+1,Pvis(2)  )~=-1)
                            r.Vc(1) = r.Vc(1)+r.Mres;
                            ronda = 0;
                        end        
                    end
                    if(Pvis(1)-1>0 && ronda == 1)
                        if(r.Flood(Pvis(1)-1,Pvis(2)  )<...
                           r.Flood(Pvis(1)  ,Pvis(2)  )&&...
                           r.Flood(Pvis(1)-1,Pvis(2)  )~=-1)
                            r.Vc(1) = r.Vc(1)-r.Mres;
                            ronda = 0;
                        end
                    end
                    if(Pvis(2)+1<size(r.Map,2) && ronda == 1)
                        if(r.Flood(Pvis(1)  ,Pvis(2)+1)<...
                           r.Flood(Pvis(1)  ,Pvis(2)  )&&...
                           r.Flood(Pvis(1)  ,Pvis(2)+1)~=-1)
                            r.Vc(2) = r.Vc(2)+r.Mres;
                            ronda = 0;
                        end
                    end
                    if(Pvis(2)-1>0  && ronda == 1)
                        if(r.Flood(Pvis(1)  ,Pvis(2)-1)<...
                           r.Flood(Pvis(1)  ,Pvis(2)  )&&...
                           r.Flood(Pvis(1)  ,Pvis(2)-1)~=-1)
                            r.Vc(2) = r.Vc(2)-r.Mres;
                        end
                    end
                    Pvis = [round(r.Vc(1))/r.Mres, round(r.Vc(2))/r.Mres];
                end
            catch
                disp('centroide fuera de rango')
            end
        end
    % Funciones de detecci�n de Obst�culos
        % funci�n de detecci�n de obst�culos
        function r = detectObstacle(r)
            Obs = inf;
            % Detect Obstacle in range
            for i = 1:1:r.MW/r.Mres % for each x pos (can be reduced to the
                                    % Sensed area)
                for j = 1:1:r.MH/r.Mres % for each y pos (can be reduced to
                                        % Sensed area)                    
                    DObs = sqrt((r.Pos(1)-i*r.Mres).^2+(r.Pos(2)-j*r.Mres).^2);
                    if(DObs <= Obs && DObs <= r.Sr && r.Map(i,j) == 2)
                        Obs = DObs;
                        ObsPos = [i*r.Mres;j*r.Mres];
                    end
                end
            end
            % If an obstacle was detected adds 
            % the potential generated by it
            if(Obs ~= inf && Obs > r.Mres)
                PotNorm = -(r.V0)/(Obs-r.Mres)^3; 
                direction = (r.Pos-ObsPos)/norm((r.Pos-ObsPos));
                r.PotNet = r.PotNet + PotNorm*direction;
            end
        end
    % Funciones de mantenimiento de la comunicaci�n
        function r = updateNeighbors(r)
            for i=1:1:size(r.Laplacian,1)
                if(r.Laplacian(r.ID,i)==-1)
                    r.NeighborsList = [r.NeighborsList i];
                end
            end
        end
        function r = conectToRobot(r,rn)
            if(r.Laplacian(r.ID,rn.ID)==-1)
                Drn = sqrt((r.Pos(1) - rn.Pos(1))^2+...
                           (r.Pos(2) - rn.Pos(2))^2);
                if(Drn/r.Cr>0.95)
                    % disp(['El robot ' num2str(r.ID) ' y el robot ' ...
                    %     num2str(rn.ID) ' est�n por desconectarse' ])
                    r.RecNet = 1;
                else
                    r.RecNet = 0;
                end
                if(Drn < r.Cr)
                    PotNorm = r.V0/(r.Cr^2-Drn^2);
                    r.PotNet = r.PotNet + PotNorm*(r.Pos-rn.Pos);
                elseif(r.ID < rn.ID)
                    disp(['El robot ' num2str(r.ID) ' y el robot ' num2str(rn.ID) ' se desconectaron' ])
                end
                if(Drn > r.Mres)
                    PotNorm = -r.V0/(Drn-r.Mres)^3;
                    r.PotNet = r.PotNet + PotNorm*(r.Pos-rn.Pos);
                elseif(r.ID < rn.ID)
                    disp(['El robot ' num2str(r.ID) ' y el robot ' num2str(rn.ID) ' se chocaron' ])
                end
            end
        end
        function r = conectToVictim(r,victim)
            Drv = sqrt((r.Pos(1) - victim.Pos(1))^2+...
                       (r.Pos(2) - victim.Pos(2))^2);
            SrMin = min(r.Sr,victim.Sr);
            if(Drv<SrMin)
                PotNorm = 1/(SrMin^2-Drv^2)-1/Drv^3;
                r.PotNet = r.PotNet + PotNorm*(r.Pos-victim.Pos);
            end
        end
        function r = cleanRobotPotential(r)
            r.PotNet = [0;0];
        end
        function r = updateNetwork(r)
            it = 0;
            itMax = 10;
            Na = size(r.Laplacian,1);
            Conver = Na;
            while(it<itMax && Conver ~= 0)
                it = it+1;
                Conver = Na;
                for i = 1:1:Na
                    Lt = r.Laplacian;
                    % Addition
                    Dmin = inf;
                    AddCan = 0;
                    for j = 1:1:Na
                        if(i~=j && L(i,j) == 0)
                            Dan = sqrt((P(1,i)-P(1,j))^2+(P(2,i)-P(2,j))^2);
                            if(Dan<Dmin)
                                AddCan = j;
                                Dmin = Dan;
                            end
                        end
                    end
                    Lt(i,i) = Lt(i,i)+1;
                    Lt(AddCan,AddCan) = Lt(AddCan,AddCan)+1;
                    Lt(i,AddCan) = -1;
                    Lt(AddCan,i) = -1;
                    % Supression
                    Dmax = 0;
                    DelCan = 0;
                    for j = 1:1:Na
                        if(i~=j && Lt(i,j) == -1)
                            Dan = sqrt((P(1,i)-P(1,j))^2+(P(2,i)-P(2,j))^2);
                            if(Dan>Dmax)
                                DelCan = j;
                                Dmax = Dan;
                            end
                        end
                    end
                    Lt(i,i) = Lt(i,i)-1;
                    Lt(DelCan,DelCan) = Lt(DelCan,DelCan)-1;
                    Lt(i,DelCan) = 0;
                    Lt(DelCan,i) = 0;
                    % Reeplace the candidates if they are valid and different
                    if(AddCan~=0 && DelCan~=0 && AddCan ~= DelCan)
                        % Verifies if the change is valid
                        lambdat = sort(eig(Lt));
                        if(lambdat(2)>0.1)
                            disp(['grafo conexo ^_^ se reemplaza ' ...
                                '(' num2str(i) ',' num2str(DelCan) ')'...
                                ' por (' num2str(i) ',' num2str(AddCan) ')'])
                            L=Lt;
                        else
                            disp('no se modificaron las conexiones')
                            Conver = Conver-1;
                        end
                    else
                        disp('no se modificaron las conexiones')
                        Conver = Conver-1;
                    end
                end
                if(Conver == 0)
                    disp(['Convergi� en la iteraci�n ' num2str(it)])
                    break
                end
            end
        end % not available yet
    % Se�al de control
        % determinaci�n de la se�al de control
        function [r,u] = controlSignal(r,dt)
            uExp = - r.Kp*(r.Pos - r.Vc);
            uPot = - r.Kar*r.PotNet;
            % detects if the robot is stuck
            if((norm(r.Pos - r.Vc)<r.Mres/sqrt(2) ||...
                norm(uExp+uPot)<r.Mres/sqrt(2)      ) &&...
                r.PerCon == 0)
                r.PerCon = 1/dt;
                r.PerSign = (-1).^randi([1,2],2,1);
                r.PerDir = rand(2,1);
                % disp([' Robot ' num2str(r.ID) ' atorado'])
            end
            if(r.PerCon>0)
                Per = r.PerSign.*round(r.V0/sqrt(2)).*r.PerDir;
                uExp = Per;
                r.PerCon = r.PerCon - 1;
                % disp([' Robot ' num2str(r.ID) ' perturbado Contador = ' num2str(r.PerCon)])
            end
            u = uExp + uPot;
            uNorm = norm(u);
            % saturates the robot input
            if(uNorm > r.V0)
                u = r.V0 * u/uNorm;
            end
            r.Vel = u;
        end
    % Din�mica de Movimiento
        % Din�mica de Movimiento seg�n entrada
        function r = move(r, u, dt)
            newPos = r.Pos + u .* dt;
            r.Pos = newPos;
        end
    end
end