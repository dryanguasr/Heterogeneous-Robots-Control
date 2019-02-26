% Initial Graph
P = [10  0 10  0 10;...
      0 10 20 30 40];
A = [0 0 0 0 1;...
     0 0 0 1 0;...
     0 0 0 1 1;...
     0 1 1 0 0;...
     1 0 1 0 0];
% A = [0 1 1 1 1;...
%      1 0 1 1 1;...
%      1 1 0 1 1;...
%      1 1 1 0 1;...
%      1 1 1 1 0];
D = diag(sum(A,2));
L = D-A;
Na = size(L,1);
lambda=sort(eig(L));
if(lambda(2)>0)
    disp('grafo conexo ^_^')
else
    disp('grafo no conexo u_u')
end
% display network
figure(1);
hold on
for i = 1:1:Na
    scatter(P(1,i),P(2,i),'f')
end
for i = 1:1:Na
    for j = 1:1:Na    
        if(i~=j && i<j && L(i,j)==-1)
            plot([P(1,i),P(1,j)],[P(2,i),P(2,j)],'b')
        end
    end
end
%% Deletion Test
DelCan=zeros(1,Na);
it = 0;
itMax = 10;
while it<itMax
    it = it + 1;
    Conver = Na;
    for i=1:1:Na;
        Dmax = 0;
        for j = 1:1:Na
            if(i~=j && L(i,j) ~= 0)
                Dan = sqrt((P(1,i)-P(1,j))^2+(P(2,i)-P(2,j))^2);
                if(Dan>Dmax)
                    DelCan(i) = j;
                    Dmax = Dan;
                end
            end
        end
    end
    for i = 1:1:Na
        Lt = L;
        if(L(i,DelCan(i))~=0)
            Lt(i,i) = Lt(i,i)-1;
            Lt(DelCan(i),DelCan(i)) = Lt(DelCan(i),DelCan(i))-1;
            Lt(i,DelCan(i)) = 0;
            Lt(DelCan(i),i) = 0;
            lambdat = sort(eig(Lt));
            if(lambdat(2)>0.01)
                disp(['grafo conexo ^_^ se borra la conexión (' num2str(i) ','...
                    num2str(DelCan(i)) ')'])
                L=Lt;
            else
                disp('grafo no conexo u_u , se conservan las conexiones')
                Conver = Conver-1;
            end
        else
            disp('la conexión candidata ya había sido borrada')
        end
    end
    if(Conver == 0)
        disp('convergió')
        break
    end
end
%% Addition Test
AddCan=zeros(1,Na);
it = 0;
itMax = 10;
while it<itMax
    it = it + 1;
    Conver = Na;
    for i=1:1:Na;
        Dmin = inf;
        for j = 1:1:Na
            if(i~=j && L(i,j) == 0)
                Dan = sqrt((P(1,i)-P(1,j))^2+(P(2,i)-P(2,j))^2);
                if(Dan<Dmin)
                    AddCan(i) = j;
                    Dmin = Dan;
                end
            end
        end
    end
    for i = 1:1:Na
        if(AddCan(i)~=0)
            if(L(i,AddCan(i))~=-1)
                L(i,i) = L(i,i)+1;
                L(AddCan(i),AddCan(i)) = L(AddCan(i),AddCan(i))+1;
                L(i,AddCan(i)) = -1;
                L(AddCan(i),i) = -1;
                disp(['se agrega la conexión (' num2str(i) ','...
                    num2str(AddCan(i)) ')'])
            else
                disp('la conexión candidata ya había sido creada')
                Conver = Conver - 1;
            end
        else
            Conver = Conver - 1;
        end
    end
    if(Conver == 0)
        disp('convergió')
        break
    end
end
%% replacement test
it = 0;
itMax = 10;
Conver = Na;
while(it<itMax && Conver ~= 0)
    it = it+1;
    Conver = Na;
    for i = 1:1:Na
        Lt = L;
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
        disp(['Convergió en la iteración ' num2str(it)])
        break
    end
end
% display network
clf
hold on
for i = 1:1:Na
    scatter(P(1,i),P(2,i),'f')
end
for i = 1:1:Na
    for j = 1:1:Na    
        if(i~=j&& Laplacian(i,j)==-1)
            plot([P(1,i),P(1,j)],[P(2,i),P(2,j)],'b')
        end
    end
end