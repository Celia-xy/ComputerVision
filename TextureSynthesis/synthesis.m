Win_size = 5;  % window size, odd integer such as 5,9,11...
%% image T1
I0 = (imread('T1.gif'));   
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
I1 = zeros(200,200);
[a,b] = size (I0);
I1((100 - floor(a/2)):(100 + ceil(a/2) -1), (100 - floor(b/2)):(100 + ceil(b/2) -1)) = I0;
I1 = inpaintl(I1, Win_size, Element, I0);

%% image T2
I0 = (imread('T2.gif'));
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
I2 = zeros(200,200);
[a,b] = size (I0);
I2((100 - floor(a/2)):(100 + ceil(a/2) -1), (100 - floor(b/2)):(100 + ceil(b/2) -1)) = I0;
I2 = inpaintl(I2, Win_size, Element, I0);

%% image T3
I0 = (imread('T3.gif'));
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
I3 = zeros(200,200);
[a,b] = size (I0);
I3((100 - floor(a/2)):(100 + ceil(a/2) -1), (100 - floor(b/2)):(100 + ceil(b/2) -1)) = I0;
I3 = inpaintl(I3, Win_size, Element, I0);

%% image T4
I0 = (imread('T4.gif'));
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
I4 = zeros(200,200);
[a,b] = size (I0);
I4((100 - floor(a/2)):(100 + ceil(a/2) -1), (100 - floor(b/2)):(100 + ceil(b/2) -1)) = I0;
I4 = inpaintl(I4, Win_size, Element, I0);

%% image T5
I0 = (imread('T5.gif'));
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
% change all 0 pixels to 1
[r,c] = find(I0 == 0); 
[m,n] = size(r);
for i = 1 : m
    r1 = r(i);
    c1 = c(i);
    I0(r1,c1) = 1;
end
I5 = zeros(200,200);
[a,b] = size (I0);
I5((100 - floor(a/2)):(100 + ceil(a/2) -1), (100 - floor(b/2)):(100 + ceil(b/2) -1)) = I0;
I5 = inpaintl(I5, Win_size, Element, I0);

%% figure
figure;
subplot(231);imshow(uint8(I1)); title('I1');
subplot(232);imshow(uint8(I2)); title('I2');
subplot(233);imshow(uint8(I3)); title('I3');
subplot(234);imshow(uint8(I4)); title('I4');
subplot(235);imshow(uint8(I5)); title('I5');
% figure; imshow(uint8(I1)); title('I1 win5');
% figure; imshow(uint8(I2)); title('I2 win5');
% figure; imshow(uint8(I3)); title('I3 win5');
% figure; imshow(uint8(I4)); title('I4 win5');
% figure; imshow(uint8(I5)); title('I5 win5');