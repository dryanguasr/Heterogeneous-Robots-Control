function [ DMap ] = displayMap( Map )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
DMap = zeros(size(Map,1),size(Map,2),3);
for i=1:1:size(Map,1)
    for j=1:1:size(Map,2)
        if(Map(i,j)==0)
            DMap(i,j,:) = [0,0,1];
        elseif(Map(i,j) == 1)
            DMap(i,j,:) = [1,1,1];
        elseif(Map(i,j) == 2)
            DMap(i,j,:) = [0,0,0];
        end
    end
end

end

