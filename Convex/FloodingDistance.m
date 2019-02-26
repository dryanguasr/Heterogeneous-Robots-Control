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
h=figure(1);
image([0 MW], [0 MH], scene,'CDataMapping','scaled')
%% Initialize the distance matrix
fDistance = MW/Mres*MH/Mres*ones(size(scene));
for i=1:1:size(scene,1)
    for j=1:1:size(scene,2)
        if(scene(i,j)~=1)
            fDistance(i,j)=-1;
        end
    end
end
P0 = [3,3]/Mres;
PF = [8,3]/Mres;
fDistance(P0(1),P0(2)) = 1;
dPile = zeros(size(scene));
cicle = 1;
MaxCicles = 5;
%% updates the distance
while(sum(sum(abs(fDistance-dPile)))~=0 && cicle < MaxCicles)
    disp(['entró en el ciclo ' num2str(cicle)])
    dPile = fDistance;
    cicle = cicle + 1;
    %up-down, left-right cicle
    for i=1:1:size(scene,1)
        for j = 1:1:size(scene,2)
            if(i-1>0)
                if(fDistance(i,j) > 0 &&...
                   fDistance(i-1,j) > fDistance(i,j) &&...
                   fDistance(i-1,j) ~= -1)
                    fDistance(i-1,j) = fDistance(i,j) + 1;
                end
            end
            if(i+1<=size(fDistance,1))
                if(fDistance(i,j) > 0 &&...
                   fDistance(i+1,j) > fDistance(i,j) &&...
                   fDistance(i+1,j) ~= -1)
                    fDistance(i+1,j) = fDistance(i,j) + 1;
                end
            end
            if(j-1>0)
                if(fDistance(i,j) > 0 &&...
                   fDistance(i,j-1) > fDistance(i,j) &&...
                   fDistance(i,j-1) ~= -1)
                    fDistance(i,j-1) = fDistance(i,j) + 1;
                end
            end
            if(j+1<=size(fDistance,2))
                if(fDistance(i,j) > 0 &&...
                   fDistance(i,j+1) > fDistance(i,j) &&...
                   fDistance(i,j+1) ~= -1)
                    fDistance(i,j+1) = fDistance(i,j) + 1;
                end
            end
        end
    end
    %down-up, right-left cicle
    for i=size(scene,1):-1:1
        for j = size(scene,2):-1:1
            if(i-1>0)
                if(fDistance(i,j)>0&&fDistance(i-1,j)>fDistance(i,j)&&fDistance(i-1,j)~=-1)
                    fDistance(i-1,j) = fDistance(i,j) + 1;
                end
            end
            if(i+1<=size(fDistance,1))
                if(fDistance(i,j)>0&&fDistance(i+1,j)>fDistance(i,j)&&fDistance(i+1,j)~=-1)
                    fDistance(i+1,j) = fDistance(i,j) + 1;
                end
            end
            if(j-1>0)
                if(fDistance(i,j)>0&&fDistance(i,j-1)>fDistance(i,j)&&fDistance(i,j-1)~=-1)
                    fDistance(i,j-1) = fDistance(i,j) + 1;
                end
            end
            if(j+1<=size(fDistance,2))
                if(fDistance(i,j)>0&&fDistance(i,j+1)>fDistance(i,j)&&fDistance(i,j+1)~=-1)
                    fDistance(i,j+1) = fDistance(i,j) + 1;
                end
            end
        end
    end 
end
for i=1:1:size(scene,1)
    for j=1:1:size(scene,2)
        if(fDistance(i,j)==MW/Mres*MH/Mres)
            fDistance(i,j) = 0;
        end
    end
end
DF = dPile(PF(1),PF(2));
Path = zeros(DF,2);
Pa = PF;
for i=1:1:4*DF
    while(true)
        try
            if(dPile(Pa(1)+1,Pa(2)) < dPile(Pa(1),Pa(2)) && dPile(Pa(1)+1,Pa(2)) ~= -1)
                Path(i,:) = [Pa(1)+1,Pa(2)];
                Pa = [Pa(1)+1,Pa(2)];
                break
            end
            if(dPile(Pa(1)-1,Pa(2)) < dPile(Pa(1),Pa(2)) && dPile(Pa(1)+1,Pa(2)) ~= -1)
                Path(i,:) = [Pa(1)-1,Pa(2)];
                Pa = [Pa(1)-1,Pa(2)];
                break
            end
            if(dPile(Pa(1),Pa(2)+1) < dPile(Pa(1),Pa(2)) && dPile(Pa(1)+1,Pa(2)) ~= -1)
                Path(i,:) = [Pa(1),Pa(2)+1];
                Pa = [Pa(1),Pa(2)+1];
                break
            end
            if(dPile(Pa(1),Pa(2)-1) < dPile(Pa(1),Pa(2)) && dPile(Pa(1)+1,Pa(2)) ~= -1)
                Path(i,:) = [Pa(1),Pa(2)-1];
                Pa = [Pa(1),Pa(2)-1];
                break
            else
                break
            end
        catch
            disp('algo salió mal')
            break
        end
    end
end
image([0 MW], [0 MH], fDistance,'CDataMapping','scaled')