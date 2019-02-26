function [ DVor ,Alpha ] = displayVor( Vor )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
DVor = zeros(size(Vor,1),size(Vor,2),3);
Alpha = 0.5*ones(size(Vor,1),size(Vor,2));
for i=1:1:size(Vor,1)
    for j=1:1:size(Vor,2)
        if(Vor(i,j) == 0)
            DVor(i,j,:) = [0,0,0];
            Alpha(i,j) = 1;
        elseif(Vor(i,j) == 1)
            DVor(i,j,:) = [1,0,0];
        elseif(Vor(i,j) == 2)
            DVor(i,j,:) = [0,0,1];
        elseif(Vor(i,j) == 3)
            DVor(i,j,:) = [0,1,0];
        elseif(Vor(i,j) == 4)
            DVor(i,j,:) = [0,1,1];
        elseif(Vor(i,j) == 5)
            DVor(i,j,:) = [1,1,0];
        end
    end
end
end

