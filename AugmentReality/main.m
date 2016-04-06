%% -------------------------------------------------------------------
%                               Part 1
%-------------------------------------------------------------------
%------- draw image points -------%
figure;
image = zeros(500,550);
imshow(image);
hold on;
c = [422,178,118,482,438,162,78,522];
r = [323,323,483,483,73,73,117,117];
plot(c,r,'wo');
title('Camera Image');

%------- generate P, 16*12 matrix -------%
Cube_points = [2,2,2; -2,2,2; -2,2,-2; 2,2,-2; 2,-2,2;-2,-2,2;-2,-2,-2;2,-2,-2];
P = zeros(16,12);
for i = 1 : 8
    cube_point = Cube_points(i,:);
    u = c(i);
    v = r(i);
    p = getp(cube_point,u,v);
    P(2*i-1:2*i,:) = p;
end
% print P
P

%------- solve Pm=0 -------%
[~,~,V] = svd(P);
M = zeros(3,4);
M(1,:) = V(1:4,12)';
M(2,:) = V(5:8,12)';
M(3,:) = V(9:12,12)';
% print M
M

%------- get Euclidean coordinates of the camera center -------%
[~,~,V] = svd(M);
Camera_center = V(1:3,4)/V(4,4);
Camera_center

%------- get M_prime -------%
M_prime = M(1:3,1:3)/M(3,3);
M_prime
    
%------- get R_X and Theta_x and N -------%
cos_x = M_prime(3,3)/sqrt(M_prime(3,3)*M_prime(3,3)+M_prime(3,2)*M_prime(3,2));
sin_x = -M_prime(3,2)/sqrt(M_prime(3,3)*M_prime(3,3)+M_prime(3,2)*M_prime(3,2));
theta_x = asin(sin_x)/pi*180
R_x = [1,0,0;0,cos_x,-sin_x;0,sin_x,cos_x]
N = M_prime*R_x

%------- get R_z and Theta_z -------%
cos_z = N(2,2)/sqrt(N(2,1)*N(2,1)+N(2,2)*N(2,2));
sin_z = -N(2,1)/sqrt(N(2,1)*N(2,1)+N(2,2)*N(2,2));
theta_z = asin(sin_z)/pi*180
R_z = [cos_z,-sin_z,0;sin_z,cos_z,0;0,0,1]

%------- compute K, focal lengths and image center coordinates -------%

K = N*R_z;
K = K/K(3,3)
alpha = K(1,1);
theta = acot(-K(1,2)/alpha);
beta = K(2,2)*sin(theta);
u_0 = K(1,3);
v_0 = K(2,3);
