function [errorValue] = errorFunc(x, A, M, m, nPoints)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes 
%R as rodriguez vector convert to rot matrix
R = x(:,1)
T = x(:,2)
R = rotationVectorToMatrix(R)
sumError = 0;
for i = 1:nPoints
    proj = A*(R*M(:, i) + T);
    proj = proj(1:2)/proj(3);
    pointError = norm(proj - m(:, i));
    sumError = sumError + pointError;
end

errorValue = sumError;
end

