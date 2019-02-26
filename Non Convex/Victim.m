classdef Victim
    properties
        % Caracter�sticas y estado de la v�ctima
        Pos = [0;0];         % Posici�n
        Vel = [0;0];         % Velocidad actual
        V0 = 1;              % Velocidad promedio [m/s]
        Sr = 1;              % Radio de Sensado [m]
        ID = 0;              % ID de la victima en la misi�n
        % Funci�nes de potencial de atracci�n y repulsi�n
        PotNet = [0;0];
        % Par�metros del Controlador
        Kar = 1;            % Constante proporcional para atracci�n/repulsi�n
        Conected = 0;
    end
    methods
    % Constructor
        function victim = Victim(ID, pos, V0, Sr, Kar)
            % Propiedades y estado del robot
            victim.ID = ID;
            victim.V0 = V0;
            victim.Sr = Sr;
            victim.Kar = Kar;
            victim.Pos = pos;
        end  
    
    % Funciones de mantenimiento de la comunicaci�n
        function victim = conectToRobot(victim,rn)
            if(victim.Conected == 0)
                Drv = sqrt((victim.Pos(1) - rn.Pos(1))^2+...
                    (victim.Pos(2) - rn.Pos(2))^2);
                if(Drv<victim.Sr)
                    victim.Conected = 1;
                    PotNorm = 1/(victim.Sr^2-Drv^2)-1/Drv^2;
                    victim.PotNet = victim.PotNet + PotNorm*(victim.Pos-rn.Pos);
                end
            end
        end
        function victim = cleanVictimPotential(victim)
            victim.PotNet = [0;0];
        end
    % Se�al de control
        % determinaci�n de la se�al de control
        function u = controlSignal(victim)
            uPot = - victim.Kar*victim.PotNet;
            uPotNorm = norm(- victim.Kar*victim.PotNet);
            if(uPotNorm>victim.V0)
                uPot = uPot*(victim.V0)/uPotNorm;
            end
            u = uPot;
        end
    % Din�mica de Movimiento
        % Din�mica de Movimiento seg�n entrada
        function victim = move(victim, u, dt)
            newPos = victim.Pos + u .* dt;
            victim.Pos = newPos;
        end
    end
end