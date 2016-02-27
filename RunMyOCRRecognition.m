% 
% RunMyOCRRecognition
% Recognize characters of a image
%
% Inputs: filename: the file name of image needs to be tested
%         locations: [N x 2] matrix of rough center coordinates of characters
%         classes: [N x 1] matrix of the corresponding class label for each character, 
%                  where N is the number of characters in the image
% Output: result: Recognition rate of this image
%
function result = RunMyOCRRecognition (filename, locations, classes)
% filename='test2.bmp';
% classes=classes_test2;
% locations=locations_test2;
%% Training
%
%---- Extract features of all characters in all training pictures----%
%
plot = 1; % could be 1 to show figures or 0 to not
Feature1 = OCR_Extract_Features(imread('a.bmp'),plot);
[i,j] = size(Feature1);
Label_1 = ones(i,1);
Features2 = OCR_Extract_Features(imread('d.bmp'),plot);
[i,j] = size(Features2);
Label_2 = 2*ones(i,1);
Features3 = OCR_Extract_Features(imread('f.bmp'),plot);
[i,j] = size(Features3);
Label_3 = 3*ones(i,1);
Features4 = OCR_Extract_Features(imread('h.bmp'),plot);
[i,j] = size(Features4);
Label_4 = 4*ones(i,1);
Features5 = OCR_Extract_Features(imread('k.bmp'),plot);
[i,j] = size(Features5);
Label_5 = 5*ones(i,1);
Features6 = OCR_Extract_Features(imread('m.bmp'),plot);
[i,j] = size(Features6);
Label_6 = 6*ones(i,1);
Features7 = OCR_Extract_Features(imread('n.bmp'),plot);
[i,j] = size(Features7);
Label_7 = 7*ones(i,1);
Features8 = OCR_Extract_Features(imread('o.bmp'),plot);
[i,j] = size(Features8);
Label_8 = 8*ones(i,1);
Features9 = OCR_Extract_Features(imread('p.bmp'),plot);
[i,j] = size(Features9);
Label_9 = 9*ones(i,1);
Features10 = OCR_Extract_Features(imread('q.bmp'),plot);
[i,j] = size(Features10);
Label_10 = 10*ones(i,1);
Features11 = OCR_Extract_Features(imread('r.bmp'),plot);
[i,j] = size(Features11);
Label_11 = 11*ones(i,1);
Features12 = OCR_Extract_Features(imread('s.bmp'),plot);
[i,j] = size(Features12);
Label_12 = 12*ones(i,1);
Features13 = OCR_Extract_Features(imread('u.bmp'),plot);
[i,j] = size(Features13);
Label_13 = 13*ones(i,1);
Features14= OCR_Extract_Features(imread('w.bmp'),plot);
[i,j] = size(Features14);
Label_14 = 14*ones(i,1);
Features15 = OCR_Extract_Features(imread('x.bmp'),plot);
[i,j] = size(Features15);
Label_15 = 15*ones(i,1);
Features16 = OCR_Extract_Features(imread('z.bmp'),plot);
[i,j] = size(Features16);
Label_16 = 16*ones(i,1);

%
%----Combine features of all characters in all training pictures----%
%
Features = [Feature1;Features2;Features3;Features4;Features5;Features6;
    Features7;Features8;Features9;Features10;Features11;Features12;
    Features13;Features14;Features15;Features16];
Labels = [Label_1;Label_2;Label_3;Label_4;Label_5;Label_6;Label_7;
    Label_8;Label_9;Label_10;Label_11;Label_12;Label_13;Label_14;Label_15;
    Label_16];

%
%----------------------------Normalization----------------------------%
%
[m,n] = size( Features);
Normalized_Features = ones(m,n); 
Means = zeros(1,n);
Standard_Deviation = zeros(1,n);
for j = 1 : n
    Means(1,j) = mean (Features(:,j));
    Standard_Deviation(1,j) = std (Features(:,j));
    for i = 1 : m 
        Normalized_Features(i,j) = (Features(i,j) - Means(1,j))/Standard_Deviation(1,j);
    end
end

%
%----------------------Recognition on training data---------------------%
%
D = dist2( Normalized_Features, Normalized_Features);
figure
imagesc(D) 
title('Distance Matrix')
[D_sorted, D_index] = sort( D,2);
[i,j] = size(D_index);
Result = zeros(i,1);
for k = 1:i
   Result(k) = Labels (D_index(k,2));
end

%
%--------------------------Confusion matrix---------------------------%
%
conf = ConfusionMatrix(Labels,Result,16);
figure
imagesc(conf) 
title('Confusion Matrix')

%% Testing
%
%----- --Extracting characters in Test_Image and normalization--------%
%
Test_Image = imread (filename);
Feature_T = OCR_Extract_Features(Test_Image,1);
[m,n] = size( Feature_T); 
for j = 1 : n
    for i = 1 : m 
        Feature_T(i,j) = (Feature_T(i,j) - Means(1,j))/Standard_Deviation(1,j);
    end
end

%
%----------------Recognize characters in Test_Image--------------------%
%
DT = dist2( Feature_T, Normalized_Features);
figure
imagesc(DT) 
title('Distance Matrix')
[DT_sorted, DT_index] = sort(DT,2);
[i,j] = size( DT_index);
Result = zeros(i,1);
for k = 1:i
%    Result(k) = Labels (DT_index(k,1)); 
     
%%%%%%%%%%%%%%   ------k-nearest neighbor begin------   
   Result_k = [];
   k_n = 20;
   for p = 1 : k_n
       for q = 1 : p
           Result_k = [Result_k;Labels(DT_index(k,k_n-p+1))];
       end    
   end
   Result(k) = mode(flipud(Result_k));   
%%%%%%%%%%%%%%   -------k-nearest neighbor end-------
end

%% Result and Evaluation
%
%-------------Show test image and recognization rate-----------------%
%
Class_Type = ['a','d','f','h','k','m','n','o','p','q','r','s','u','w','x','z'];
[N, M] = size(locations);
th = 200;
im2 = Test_Image;
im2(Test_Image >= th) = 0;
im2(Test_Image < th) = 1;
TL = bwlabel(im2);
Nc = max(max(TL));
figure
imagesc(TL)
title('Test Image')
num = 0;
hit = 0;
Result_Order = zeros(N,1);
for i = 1 : Nc
    [r,c] = find( TL == i);
    maxr = max(r);
    minr = min(r);
    maxc = max(c);
    minc = min(c);
    if ((maxr - minr)<80 && (maxr - minr)>7 && (maxc - minc)<80  && (maxc - minc)>7 && (maxr-minr)*(maxr-minr)>70)
        num = num + 1;
        rectangle('Position',[minc,minr,maxc-minc+1,maxr-minr+1], 'EdgeColor','w');
        text(maxc, maxr+10, Class_Type( Result(num)),'color','w');
        for j = 1 : N
            if (locations(j,1)>minc && locations(j,1)<maxc && locations(j,2)>minr && locations(j,2)<maxr)
               Result_Order(j,1) = Result(num);
               text(minc, maxr+10, Class_Type(classes(j)),'color','w');
               if Result(num) == classes(j);
                   hit = hit + 1;
               end
            end
        end
    end    
end
Num_Component = num
Recognization_Rate = hit/N
result = Recognization_Rate;
