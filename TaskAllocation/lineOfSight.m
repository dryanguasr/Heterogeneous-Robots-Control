function [ LineOfSight ] = lineOfSight( scene, Mres, pos )
%lineOfSight determies the visibility for each point from a determined
%position in a given scene
%   determies the visibility for each point from a determined
%   position in a given scene using radial rays to determine if the point
%   can or can not be reached linearly from the robot perspective
LineOfSight = zeros(size(scene));
for i=1:1:size(scene,1)
    for j=1:1:size(scene,2)
        if(scene(i,j)~=1)
            LineOfSight(i,j)=-1;
        end
    end
end
P0 = pos'/Mres;
if(P0(1) == 0)
    P0(1) = 1;
end
if(P0(2) == 0)
    P0(2) = 1;
end
%% line Of Sight Calculus
for x = 1:1:size(scene,1)
    for y = 1:1:size(scene,2)
        dP = P0 - [x,y];
        if(norm(dP)==0)
            % evaluating itself
            LineOfSight(x,y) = 1;
        elseif(LineOfSight(x,y) ~= 1)
            % general evaluation
            continua = 1;
            rho = 0:1/(abs(dP(1))+abs(dP(2))):1;
            cur = [P0(1).*(1-rho) + x*(rho);...
                   P0(2).*(1-rho) + y*(rho)];
            for i = 1:1:size(rho,2)
                PCur = [max(round(cur(1,i)),1) , max(round(cur(2,i)),1)];
                if(scene(PCur(1),PCur(2)) == 1 && continua == 1)
                    LineOfSight(PCur(1),PCur(2)) = 1;
                elseif(continua == 1)
                    LineOfSight(PCur(1),PCur(2)) = 1;
                    % continua = 0;
                    break
                else
                    LineOfSight(PCur(1),PCur(2)) = -1;
                end
            end
        end
    end
end
end