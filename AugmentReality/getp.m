function p = getp(cube_point,u,v)
% function: get P of one cube corner and its image coordinates
% input: cube_point  coordinate of a cube corner, 1*3 matrix
%        u           x-coordinate of corresponding image
%        v           y-coordinate of corresponding image 
% output:p           matrix P, 2*12 matrix
    P = [cube_point,1];
    p = zeros(2,12);
    p(1,1:4) = P;
    p(2,5:8) = P;
    p(1,9:12) = -u*P;
    p(2,9:12) = -v*P;