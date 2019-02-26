% Line of Sight Based on - https://www.youtube.com/watch?v=Qwv_c1Oc78w
%% Scene Definition
MW = 10;%m
MH = 10;%m
Mres = .1; % [m/celda]
% creates the scene empty (1 = empty space)
scene = ones(MW/Mres,MH/Mres);
% creates a non accessible areas
% scene (2 = non accessible).
for i = 1:1:round(.8*size(scene,1))
    for j = round(0.45*MH/Mres):1:round(0.55*MH/Mres)
        scene(i,j) = 2;
    end
end
for i = round(0.45*MW/Mres):1:round(0.55*MW/Mres)
    for j = 1:1:.3*size(scene,1)
        scene(i,j) = 2;
    end
end
for i = round(0.45*MW/Mres):1:round(0.55*MW/Mres)
    for j = round(.7*size(scene,1)):1:size(scene,1)
        scene(i,j) = 2;
    end
end
% h=figure(1);
image([0 MW], [0 MH], scene,'CDataMapping','scaled')
%% line of Sight initialization
lSight = zeros(size(scene));
for i=1:1:size(scene,1)
    for j=1:1:size(scene,2)
        if(scene(i,j)~=1)
            lSight(i,j)=-1;
        end
    end
end
P0 = [7,2]/Mres;
%% line Of Sight Calculus
for x = 1:1:size(scene,1)
    for y = 1:1:size(scene,2)
        dP = P0 - [x,y];        
        if(norm(dP)==0)
            % evaluating itself
            lSight(x,y) = 1;
        else
            % general evaluation
            continua = 1;
            rho = 0:1/(abs(dP(1))+abs(dP(2))):1;
            cur = [P0(1).*(1-rho) + x*(rho);...
                   P0(2).*(1-rho) + y*(rho)];
            for i = 1:1:size(rho,2)                
                if(scene(round(cur(1,i)),round(cur(2,i))) == 1 && continua == 1)
                    lSight(round(cur(1,i)),round(cur(2,i))) = 1;
                else
                    lSight(round(cur(1,i)),round(cur(2,i))) = 1;
                    break
                end
            end
        end
    end
end
h = figure(1);
image([0 MW], [0 MH], lSight,'CDataMapping','scaled')
saveas(h,'LineOfSight.jpg')