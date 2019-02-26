classdef Victim
    properties
        % Características y estado de la víctima
        Pos = [0;0];         % Posición
        Vel = [0;0];         % Velocidad actual
        V0 = 1;              % Velocidad promedio [m/s]
        Sr = 1;              % Radio de Sensado [m]
        ID = 0;              % ID de la victima en la misión
        % Funciónes de potencial de atracción y repulsión
        PotNet = [0;0];
        % Parámetros del Controlador
        Kar = 1;            % Constante proporcional para atracción/repulsión
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
    
    % Funciones de mantenimiento de la comunicación
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
    % Señal de control
        % determinación de la señal de control
        function u = controlSignal(victim)
            uPot = - victim.Kar*victim.PotNet;
            uPotNorm = norm(- victim.Kar*victim.PotNet);
            if(uPotNorm>victim.V0)
                uPot = uPot*(victim.V0)/uPotNorm;
            end
            u = uPot;
        end
    % Dinámica de Movimiento
        % Dinámica de Movimiento según entrada
        function victim = move(victim, u, dt)
            newPos = victim.Pos + u .* dt;
            victim.Pos = newPos;
        end
    end
end