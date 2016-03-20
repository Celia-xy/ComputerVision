
%% leung's algo to remolve person
I0 = (imread('test_im31.jpg'));
I1 = rgb2gray(I0);
Win_size = 9;                          % window size, odd integer such as 5,9,11...
Element = [0 0 0; 1 1 0; 0 1 0];       % the element for dilation
% change all 0 pixels to 1
[r,c] = find(I1 == 0); 
[m,n] = size(r);
for i = 1 : m
    r1 = r(i);
    c1 = c(i);
    I1(r1,c1) = 1;
end
% remove the person
c1 = [81   81   94   89   94   94   87   87];
r1 = [127   175   175   161   149   138   133   127];
BW = roipoly(I1,c1,r1);
BW = 1 - uint8(BW);                   % 0s unfilled region
I1 = I1.*(BW);                        % black unfilled region
% implement leung's algorithm
Il1 = inpaintl(I1, Win_size, Element, I1);
clf; 
imshow(Il1);                          % man-filled image
title('(Leung) Man-filled');

%% leung's algo to remolve the sign

Win_size = 7;                         % window size, odd integer such as 5,9,11...
Element = [0 0 0; 0 1 1; 0 0 0];      % the element for dilation
% change all 0 pixels to 1
I1 = Il1;
[r,c] = find(I1 == 0); 
I1(r,c) = 1;
% remove the sign
c2 = [286   285   279   285   300   294   289   289];
r2 = [240   204   199   185   189   206   205   240];
BW = roipoly(I1,c2,r2);
BW = 1 - uint8(BW);                   % 0s unfilled region
I1 = I1.*(BW);                        % black unfilled region
% implement leung's algorithm
Il2 = inpaintl(I1, Win_size, Element, I1);
clf; 
imshow(Il2);                          % filled image
title('(Leung) Sign-filled');

%% leung's algo to fix ground

Win_size = 7;                         % window size, odd integer such as 5,9,11...
Element = [0 0 0; 1 1 0; 0 0 0];      % the element for dilation
% change all 0 pixels to 1
I1 = Il2;
[r,c] = find(I1 == 0); 
I1(r,c) = 1;
% remolve the ground
c3 = [0   203   244   145   0];
r3 = [227   159   159   240   240];
BW = roipoly(I1,c3,r3);
BW = 1 - uint8(BW);                   % 0s unfilled region
I1 = I1.*(BW);                        % black unfilled region
% define sample area
c = [222   142   284   256];
r = [175   240   240   175];
BW = uint8(roipoly(I1,c,r));
Sample_area = I1 .* BW;
% implement leung's algorithm
imshow(I1);
imshow(Sample_area);
Il3 = inpaintl(I1, Win_size, Element, Sample_area);
clf; 
imshow(Il3);                          % filled image
title('(Leung) Ground-filled');

%% criminisi's algo to remolve person
B = double(I0(:,:,3));
G = double(I0(:,:,2));
R = double(I0(:,:,1));
% green unfilled region
BW = double(roipoly(I0,c1,r1));    % 1s unfilled region    
R = R .* ~BW;
G = G .* ~BW + 255 * BW;
B = B .* ~BW;
I2 = uint8(cat(3,R,G,B));
% implement criminisi's algorithm
[Ic1,i2,i3,c,d,mov] = inpaintc(I0,I2,[0 255 0]);
Ic1 = uint8(Ic1);
imshow(Ic1);                        % filled image
title ('(criminisi) Man-filled');

%% criminisi's algo to remolve sign
I2 = Ic1;
B = double(I2(:,:,3));
G = double(I2(:,:,2));
R = double(I2(:,:,1));
% green unfilled region
BW = double(roipoly(I0,c2,r2));     % 1s unfilled region    
R = R .* ~BW;
G = G .* ~BW + 255 * BW;
B = B .* ~BW;
I2 = uint8(cat(3,R,G,B));
% implement criminisi's algorithm
[Ic2,i2,i3,c,d,mov] = inpaintc(Ic1,I2,[0 255 0]);
Ic2 = uint8(Ic2);
imshow(Ic2);  % filled image
title ('(criminisi) Sign-filled');

% %% criminisi's algo to fix ground(initial)
% I3 = Ic2;
% B = double(I3(:,:,3));
% G = double(I3(:,:,2));
% R = double(I3(:,:,1));
% % green unfilled region
% BW = double(roipoly(I0,c3,r3));     % 1s unfilled region    
% R = R .* ~BW;
% G = G .* ~BW + 255 * BW;
% B = B .* ~BW;
% I3 = uint8(cat(3,R,G,B));
% % implement criminisi's algorithm
% [Ic3,i2,i3,c,d,mov] = inpaintc(Ic2,I3,[0 255 0]);
% Ic3 = uint8(Ic3);
% figure;
% imshow(uint8(Ic3)); title('(Criminisi) Ground-filled(initial)');

%% criminisi's algo to fix ground(improved)
% define image area
I2 = Ic2;
c = [0   208   263   276   0];
r = [222   159   159   240   240];
BW = double(roipoly(I2,c,r));
B = double(I2(:,:,3));
G = double(I2(:,:,2));
R = double(I2(:,:,1));
R1 = R .* BW;
G1 = G .* BW;
B1 = B .* BW;
Image_area = uint8(cat(3,R1,G1,B1));
% green unfilled region
BW2 = double(roipoly(I2,c3,r3));     % 1s unfilled region    
R1 = R1 .* ~BW2;
G1 = G1 .* ~BW2 + 255 * (BW2 + ~BW );
B1 = B1 .* ~BW2;
I2 = uint8(cat(3,R1,G1,B1));
% implement criminisi's algorithm
[Ic3,i2,i3,c,d,mov] = inpaintc(Image_area,I2,[0 255 0]);
B2 = double(Ic3(:,:,3));
G2 = double(Ic3(:,:,2));
R2 = double(Ic3(:,:,1));
R2 = R2 .* BW + R .* ~BW;
G2 = G2 .* BW + G .* ~BW;
B2 = B2 .* BW + B .* ~BW;
Ic3 = uint8(cat(3,R2,G2,B2));
imshow(Ic3);                         % filled image
title ('(criminisi) Ground-filled(improved)');
%% output
clf;
figure;
subplot(321);imshow(uint8(Il1)); title('(Leung) Man-filled');
subplot(323);imshow(uint8(Il2)); title('(Leung) Sign-filled');
subplot(325);imshow(uint8(Il3)); title('(Leung) Ground-filled');
subplot(322);imshow(uint8(Ic1)); title('(Criminisi) Man-filled');
subplot(324);imshow(uint8(Ic2)); title('(Criminisi) Sign-filled');
subplot(326);imshow(uint8(Ic3)); title('(Criminisi) Ground-filled');
% figure; imshow(uint8(Il1)); title('(Leung) Man-filled');
% figure; imshow(uint8(Il2)); title('(Leung) Sign-filled');
% figure; imshow(uint8(Il3)); title('(Leung) Ground-filled');
% figure; imshow(uint8(Ic1)); title('(Criminisi) Man-filled');
% figure; imshow(uint8(Ic2)); title('(Criminisi) Sign-filled');
% figure; imshow(uint8(Ic3)); title('(Criminisi) Ground-filled(improved)');