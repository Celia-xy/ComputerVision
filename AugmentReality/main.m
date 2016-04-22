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

%------------------- improving accuracy -------------------%
% ---------- get approximate grid corners
P_80 = 0.03*[0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9, 0,1,2,3,4,5,6,7,8,9;
    0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1, 2,2,2,2,2,2,2,2,2,2, 3,3,3,3,3,3,3,3,3,3, 4,4,4,4,4,4,4,4,4,4, 5,5,5,5,5,5,5,5,5,5, 6,6,6,6,6,6,6,6,6,6, 7,7,7,7,7,7,7,7,7,7];
P_80(3,:) = ones(1,80);
p_approx = zeros(3,80,4);
figure; 
for i = 1 : 4
    p_approx(:,:,i) = H(:,:,i)*P_80;
    p_approx(:,:,i) = p_approx(:,:,i)./[p_approx(3,:,i);p_approx(3,:,i);p_approx(3,:,i)];
    subplot(2,2,i); imshow(eval(['I',int2str(i)])); 
    hold on;
    c = uint16(p_approx(1,:,i));
    r = uint16(p_approx(2,:,i));
    plot(c,r,'go');
    title(['Figure1:Projected grid corners ',int2str(i)]);
end

% ---------- detect Harris corners
figure; 
for i = 1 : 4     
    [cim, r, c, rsubp, csubp] = harris(rgb2gray(eval(['I',int2str(i)])), 2, 500, 2, 0);
    subplot(2,2,i); imshow(eval(['I',int2str(i)])); 
    hold on;
    plot(c,r,'rx');
    title(['Figure2:Harris corners ',int2str(i)]);
    
    % -------- compute p_correct  
    N = dist2([csubp,rsubp], p_approx(1:2,:,i)');
    for j = 1 : 80
        Ni = N(:,j);
        n = find(Ni == min(Ni));
        p_correct(1,j,i) = c(n);
        p_correct(2,j,i) = r(n);
    end
end
% print p_correct
figure;
for i = 1 : 4
    subplot(2,2,i); imshow(eval(['I',int2str(i)])); 
    hold on;
    plot(p_correct(1,:,i),p_correct(2,:,i),'rx');
    title(['Figure3:grid points ',int2str(i)]);
end
p_correct(3,:,:) = ones(1,80,4);

% -------- compute new homography H
for i = 1 : 4
    h = homography2d(P_80, p_correct(:,:,i));
    s = h*P_80;
    H_new(:,:,i) = h;
    v11 = [h(1,1)*h(1,1),h(1,1)*h(2,1)+h(2,1)*h(1,1),h(2,1)*h(2,1),h(3,1)*h(1,1)+h(1,1)*h(3,1),h(3,1)*h(2,1)+h(2,1)*h(3,1),h(3,1)*h(3,1)]';
    v12 = [h(1,1)*h(1,2),h(1,1)*h(2,2)+h(2,1)*h(1,2),h(2,1)*h(2,2),h(3,1)*h(1,2)+h(1,1)*h(3,2),h(3,1)*h(2,2)+h(2,1)*h(3,2),h(3,1)*h(3,2)]';
    v22 = [h(1,2)*h(1,2),h(1,2)*h(2,2)+h(2,2)*h(1,2),h(2,2)*h(2,2),h(3,2)*h(1,2)+h(1,2)*h(3,2),h(3,2)*h(2,2)+h(2,2)*h(3,2),h(3,2)*h(3,2)]';
    V_0_new(2*i-1,:) = v12';
    V_0_new(2*i,:) = v11'-v22';    
end
H_new
% ---------- compute new K, R, t
% compute new B
[~,~,V] = svd(V_0_new);
B_new = zeros(3,3);
B_new(1,1) = V(1,6);
B_new(1,2) = V(2,6);
B_new(2,1) = V(2,6);
B_new(2,2) = V(3,6);
B_new(1,3) = V(4,6);
B_new(3,1) = V(4,6);
B_new(2,3) = V(5,6);
B_new(3,2) = V(5,6);
B_new(3,3) = V(6,6);
% compute new A (or K)
v_0_2d_new = (B_new(1,2)*B_new(1,3)-B_new(1,1)*B_new(2,3))/(B_new(1,1)*B_new(2,2)-B_new(1,2)*B_new(1,2));
lambda_2d_new = B_new(3,3)-(B_new(1,3)*B_new(1,3)+v_0_2d_new*(B_new(1,2)*B_new(1,3)-B_new(1,1)*B_new(2,3)))/B_new(1,1);
alpha_2d_new = sqrt(lambda_2d_new/B_new(1,1));
beta_2d_new = sqrt(lambda_2d_new*B_new(1,1)/(B_new(1,1)*B_new(2,2)-B_new(1,2)*B_new(1,2)));
gamma_2d_new = -B_new(1,2)*alpha_2d_new*alpha_2d_new*beta_2d_new/lambda_2d_new;
u_0_2d_new = gamma_2d_new*v_0_2d_new/alpha_2d_new - B_new(1,3)*alpha_2d_new*alpha_2d_new/gamma_2d_new;
A_new = [alpha_2d_new,gamma_2d_new,u_0_2d_new;0,beta_2d_new,v_0_2d_new;0,0,1];
A_inv = eye(3)/A_new;
R_new = zeros(3,3,4);
t_new = zeros(3,4);
% computer R and t
 for i = 1:4  
    R_new(:,:,i) = A_inv * H_new(:,:,i);
    t_new(:,i) = R_new(:,3,i);
    R_new(:,3,i) = cross(R_new(:,1,i),R_new(:,2,i));

 end

% print new R, A and t
t_new
R_new
A_new

% ----------------- compute error -----------------
p_projected = zeros(3,80,4);
for i = 1 : 4
    % get projected corners
    p_projected(:,:,i) = H_new(:,:,i)*P_80;
    p_projected(:,:,i) = p_projected(:,:,i)./[p_projected(3,:,i);p_projected(3,:,i);p_projected(3,:,i)];
    % compute previous error
    err_previous(:,i) = (p_approx(1,:,i) - p_correct(1,:,i))*(p_approx(1,:,i) - p_correct(1,:,i))' + (p_approx(2,:,i) - p_correct(2,:,i))*(p_approx(2,:,i) - p_correct(2,:,i))';
    % compute improved error
    err_reprojection(:,i) = (p_projected(1,:,i) - p_correct(1,:,i))*(p_projected(1,:,i) - p_correct(1,:,i))' + (p_projected(2,:,i) - p_correct(2,:,i))*(p_projected(2,:,i) - p_correct(2,:,i))';
end

%% -------------------------------------------------------------------
%                               Part 3
%-------------------------------------------------------------------
%------------------ Augmenting an Image ----------------
% get world coordinates of the clip art image  
I0 = rgb2gray(imread('1.jpg'));
I1 = imread('1.gif');
figure;
imshow(I0);
figure;
imshow(I1);

[c,r,~] = size(I0);
d = 0.21/(c-1)/2;
X_1 = zeros(3,c*r);
X_0 = zeros(2,c*r);
for i = 1 : c
    for j = 1 : r
        X_0(1,(i-1)*r+j) = j;
        X_1(1,(i-1)*r+j) = d*(j-1);
        X_0(2,(i-1)*r+j) = i;
        X_1(2,(i-1)*r+j) = d*(i-1);
        X_1(3,(i-1)*r+j) = 1;
    end
end

%----------- transfer to image coordinates
I0_projected = ones(480,640,4)*255; 
for i = 1 : 4
    % transfer
    X_2(:,:,i) = H_new(:,:,i) * X_1;
    X_2(:,:,i) = X_2(:,:,i)./[X_2(3,:,i);X_2(3,:,i);X_2(3,:,i)];
    X_2(:,:,i) = uint16(X_2(:,:,i));
    for j = 1 : r * c
        if (I0(c-X_0(2,j)+1,X_0(1,j)) < 255)
            I0_projected(X_2(2,j,i),X_2(1,j,i),i) = I0(c-X_0(2,j)+1,X_0(1,j));
        end
    end  
end
% print projected images
figure;
for k = 1 : 4
    % I0_projected(:,:,k) = imerode(I0_projected(:,:,k),ones(3,3));
    B = double(eval(['I',int2str(k),'(:,:,3)']));
    G = double(eval(['I',int2str(k),'(:,:,2)']));
    R = double(eval(['I',int2str(k),'(:,:,1)']));
    for i = 1 : 480
        for j = 1 : 640
            if (I0_projected(i,j,k)<255)
                B(i,j) = I0_projected(i,j,k);
                G(i,j) = I0_projected(i,j,k);
                R(i,j) = I0_projected(i,j,k);
            end
        end
    end
    B = medfilt2(B);
    G = medfilt2(G);
    R = medfilt2(R);
    I_projected = uint8(cat(3,R,G,B));   
    subplot(2,2,k);
    imshow(I_projected); 
    title(['Figure4:Augmenting image ',int2str(k)]);
end
% ----------- Augmenting an Object ----------------
% world coordinates of cube
X1_cube = [0, 0.09, 0.09, 0,    0,    0.09, 0.09, 0;
           0, 0,    0.09, 0.09, 0,    0,    0.09, 0.09;
           0, 0,    0,    0,    0.09, 0.09, 0.09, 0.09;
           1, 1,    1,    1,    1,    1,    1,    1];
% transfer and print
X2_cube = zeros(3,8,4);
H_cube = zeros(3,4,4);
figure;
for i = 1 : 4
    H_cube(:,:,i) = A_new * [R_new(:,:,i),t_new(:,i)];
    X2_cube(:,:,i) = H_cube(:,:,i) * X1_cube;
    X2_cube(:,:,i) = X2_cube(:,:,i)./[X2_cube(3,:,i);X2_cube(3,:,i);X2_cube(3,:,i)];
    subplot(2,2,i); imshow(eval(['I',int2str(i)])); 
    hold on;
    c = X2_cube(2,:,i);
    r = X2_cube(1,:,i);
    plot([r(1:4),r(1),r(5:8),r(5)],[c(1:4),c(1),c(5:8),c(5)],'go-');
    plot([r(2),r(6)],[c(2),c(6)],'go-');
    plot([r(3),r(7)],[c(3),c(7)],'go-');
    plot([r(4),r(8)],[c(4),c(8)],'go-');
    title(['Figure5:Augmenting object',int2str(i)]);
end
