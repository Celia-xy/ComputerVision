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

%% -------------------------------------------------------------------
%                               Part 2
%-------------------------------------------------------------------
%------- corner extraction and homography computation -------%
w = 30*9/1000;
h = 30*7/1000;
x_1 = [0,w,w,0;0,0,h,h;1,1,1,1];
I1 = imread('images2.png');
I2 = imread('images9.png');
I3 = imread('images12.png');
I4 = imread('images20.png');
V_0 = zeros(8,6);
figure;
for i = 1 : 4
    imshow(eval(['I',int2str(i)]));
    [x,y] = ginput(4);
    x_2(1,:) = x';
    x_2(2,:) = y';
    x_2(3,:) = ones(1,4);
    h = homography2d(x_1,x_2);
    s = h*x_1;
    H(:,:,i) = h;
    v11 = [h(1,1)*h(1,1),h(1,1)*h(2,1)+h(2,1)*h(1,1),h(2,1)*h(2,1),h(3,1)*h(1,1)+h(1,1)*h(3,1),h(3,1)*h(2,1)+h(2,1)*h(3,1),h(3,1)*h(3,1)]';
    v12 = [h(1,1)*h(1,2),h(1,1)*h(2,2)+h(2,1)*h(1,2),h(2,1)*h(2,2),h(3,1)*h(1,2)+h(1,1)*h(3,2),h(3,1)*h(2,2)+h(2,1)*h(3,2),h(3,1)*h(3,2)]';
    v22 = [h(1,2)*h(1,2),h(1,2)*h(2,2)+h(2,2)*h(1,2),h(2,2)*h(2,2),h(3,2)*h(1,2)+h(1,2)*h(3,2),h(3,2)*h(2,2)+h(2,2)*h(3,2),h(3,2)*h(3,2)]';
    V_0(2*i-1,:) = v12';
    V_0(2*i,:) = v11'-v22';    
end
H
%------- compute intrinsic and extrinsic parameters -------%
% compute B
[~,~,V] = svd(V_0);
B = zeros(3,3);
B(1,1) = V(1,6)';
B(1,2) = V(2,6)';
B(2,1) = V(2,6)';
B(2,2) = V(3,6)';
B(1,3) = V(4,6)';
B(3,1) = V(4,6)';
B(2,3) = V(5,6)';
B(3,2) = V(5,6)';
B(3,3) = V(6,6)';
B

% compute intrinsic parameters and A
v_0_2d = (B(1,2)*B(1,3)-B(1,1)*B(2,3))/(B(1,1)*B(2,2)-B(1,2)*B(1,2))
lambda_2d = B(3,3)-(B(1,3)*B(1,3)+v_0_2d*(B(1,2)*B(1,3)-B(1,1)*B(2,3)))/B(1,1)
alpha_2d = sqrt(lambda_2d/B(1,1))
beta_2d = sqrt(lambda_2d*B(1,1)/(B(1,1)*B(2,2)-B(1,2)*B(1,2)))
gamma_2d = -B(1,2)*alpha_2d*alpha_2d*beta_2d/lambda_2d
u_0_2d = gamma_2d*v_0_2d/alpha_2d - B(1,3)*alpha_2d*alpha_2d/gamma_2d
A = [alpha_2d,gamma_2d,u_0_2d;0,beta_2d,v_0_2d;0,0,1];
A_inv = eye(3)/A;
R_0 = zeros(3,3,4);
R = zeros(3,3,4);
t = zeros(3,4);

% compute extrinsic parameters and A
for i = 1:4
    % computer R and t
    lambda = 1/((A_inv*H(1:3,1,i))'*(A_inv*H(1:3,1,i)));
    R_0(1:3,1,i) = A_inv*H(1:3,1,i)*lambda;
    R_0(1:3,2,i) = A_inv*H(1:3,2,i)*lambda;
    R_0(1:3,3,i) = cross(R_0(1:3,1,i),R_0(1:3,2,i));
    t(:,i) = A_inv*H(1:3,3,i)*lambda;
    % test R
    test1(:,:,i) = R_0(:,:,i)'*R_0(:,:,i);
    % improve R
    [U,~,V] = svd(R_0(:,:,i));
    R(:,:,i) = U*V';
    % test R
    test2(:,:,i) = R(:,:,i)'*R(:,:,i);
end
% print t,R_0,R,test1=R_0'R_0,test2=R'R
t
R_0
R
test1
test2
