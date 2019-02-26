%% Adds all the posible conections
Nr = size(Laplacian,1);
AddCan=zeros(1,Nr);
it = 0;
itMax = 10;
while it<itMax
    it = it + 1;
    Conver = Nr;
    for i=1:1:Nr;
        Dmin = robot{r}.Cr;
        for j = 1:1:Nr
            if(i~=j && Laplacian(i,j) ~= -1)
                Dan = sqrt((robot{i}.Pos(1)-robot{j}.Pos(1))^2+...
                           (robot{i}.Pos(2)-robot{j}.Pos(2))^2 );
                if(Dan < Dmin)
                    AddCan(i) = j;
                    Dmin = Dan;
                end
            end
        end
    end
    for i = 1:1:Nr
        if(AddCan(i)~=0)
            if(Laplacian(i,AddCan(i))~=-1)
                Laplacian(i,i) = Laplacian(i,i)+1;
                Laplacian(AddCan(i),AddCan(i)) = Laplacian(AddCan(i),AddCan(i))+1;
                Laplacian(i,AddCan(i)) = -1;
                Laplacian(AddCan(i),i) = -1;
                % disp(['se agrega la conexión (' num2str(i) ','...
                %        num2str(AddCan(i)) ')'])
            else
                % disp('la conexión candidata ya había sido creada')
                Conver = Conver - 1;
            end
        else
            Conver = Conver - 1;
        end
    end
    if(Conver == 0)
        % disp('convergió')
        break
    end
end
%% Delete the non necesary conections
DelCan=zeros(1,Nr);
it = 0;
itMax = 10;
while it<itMax
    it = it + 1;
    Conver = Nr;
    for i=1:1:Nr;
        Dmax = 0;
        for j = 1:1:Nr
            if(i~=j && Laplacian(i,j) ~= 0)
                Dan = sqrt((robot{i}.Pos(1)-robot{j}.Pos(1))^2+...
                           (robot{i}.Pos(2)-robot{j}.Pos(2))^2 );
                if(Dan>Dmax)
                    DelCan(i) = j;
                    Dmax = Dan;
                end
            end
        end
    end
    for i = 1:1:Nr
        Lt = Laplacian;
        if(Laplacian(i,DelCan(i))~=0)
            Lt(i,i) = Lt(i,i)-1;
            Lt(DelCan(i),DelCan(i)) = Lt(DelCan(i),DelCan(i))-1;
            Lt(i,DelCan(i)) = 0;
            Lt(DelCan(i),i) = 0;
            lambdat = sort(eig(Lt));
            if(lambdat(2)>0.01)
                % disp(['grafo conexo ^_^ se borra la conexión (' num2str(i) ','...
                %       num2str(DelCan(i)) ')'])
                Laplacian=Lt;
            else
                % disp('grafo no conexo u_u , se conservan las conexiones')
                Conver = Conver-1;
            end
        else
            % disp('la conexión candidata ya había sido borrada')
        end
    end
    if(Conver == 0)
        % disp('convergió')
        break
    end
end
