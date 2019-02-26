Nr = 3;
Nt = 5;
T = 40*rand(2,Nt);
RP = 40*rand(2,Nr);
w = zeros(Nr,Nt);
for i = 1:1:Nr
    for j = 1:1:Nt
        w(i,j) = sqrt((T(1,j)-RP(1,i))^2+(T(2,j)-RP(2,i))^2);
    end
end
A = zeros(Nr,Nt);
for i = 1:1:Nr
    Vmin = inf;
    for j = 1:1:Nt
        V = w(i,j);
        if(V < Vmin)
            A(i,:) = zeros(1,Nt);
            A(i,j) = 1;
            Vmin = V;
        end
    end
end
J = sum(sum(w.*A));