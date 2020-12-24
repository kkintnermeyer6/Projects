clear all; close all; clc;

%% Cropped Images ---------------------------------------------------------------------
% mainfolder = 'C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces_cropped\CroppedYale';
% DTest = dir(mainfolder);
% DTest(1).name;
%z_pic = imread("C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces_cropped\CroppedYale\yaleB01\yaleB01_P00A+000E+00.pgm","pgm");
z_pic = imresize(imread("C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces_cropped\CroppedYale\yaleB01\yaleB01_P00A+000E+00.pgm","pgm"),[120 80]);
%imshow(ans)


% Reading in images
count = 0;
%AllImages = zeros(size(z_pic,1),size(z_pic,2),2432);
%ImagesColumns = zeros(size(z_pic,1)*size(z_pic,2),1);

%Run once to read in the data and save to a .mat file to avoid slowdowns
%when rerunning code.
% for j = 1:39
%     if j == 14
%         continue
%     end
%     temp = "";
%     if j < 10
%         temp = "0";
%     end
%     folder = "yaleB" + temp;
%     folder = folder + j;
%     currentfolder = 'C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces_cropped\CroppedYale\' + folder;
%     D = dir(currentfolder);
%     for k = 3:66
%         strName = D(k).name;
%         count = count+1;
%         %str_trim = strName(1:strfind(strName,'.')-1);
%         currentfile = currentfolder + "\" + strName;
%         A = imresize(double(imread(currentfile,"pgm")),[120 80]);
%         AllImages(:,:,count) = A;
%         %C = reshape(imresize(double(imread(currentfile,"pgm")),[120 80]),size(z_pic,1)*size(z_pic,2),1);
%         %ImagesColumns = [ImagesColumns,C];
%         
%     end
% end
%save AllImages to AllImagesStacked.mat


%Run once to reshape data into matrix and save to .mat file to avoid
%slowdowns when running code.
% load("AllImagesStacked");
% z_A = AllImages(:,:,1);
% C_ColumnPic = zeros(size(z_A,1)*size(z_A,2),2432);
% for j = 1:2432
%     A = AllImages(:,:,j);
%     A = reshape(A,size(A,1)*size(A,2),1);
%     C_ColumnPic(:,j) = A;
% end


% load("PicsInColumns");
% Corr_Matrix = C_ColumnPic*C_ColumnPic';


% load("CorrelationMatrix")
% [U,S,V] = svd(Corr_Matrix,'econ');
% [v_eigs,d_eigs] = eigs(Corr_Matrix,20,'lm');

% Avg Faces calculation
% load("PicsInColumns");
% AvgFaces = zeros(size(C_ColumnPic,1),size(C_ColumnPic,2)/64);
% for j = 1:38
%     temp = zeros(size(C_ColumnPic,1),1);
%     for k = 0:63
%         temp = temp + C_ColumnPic(:,j+k);
%     end
%     temp = temp*(1/64);
%     AvgFaces(:,j) = temp;
% end

% Uncomment here ------------------------------------
% Pulling just one copy of each face
load("PicsInColumns");
SingleFaces = zeros(size(C_ColumnPic,1),size(C_ColumnPic,2)/64);
for j = 1:38
    SingleFaces(:,j) = C_ColumnPic(:,(j-1)*64+1);
end


load("eigenDecomp")
load("SVD")
S_prime = diag(S);
S_prime = S_prime(1:100,1);
f1 = figure;
plot(S_prime,'r.');
f2 = figure;
semilogy(S_prime,'r.');

numEigenfaces = 150;
U_firstTwenty = U(:,1:numEigenfaces);
U_firstSix = U(:,1:6);
f3 = figure;

for k = 1:size(U_firstSix,2)
    U_small = U_firstSix(:,k);
    U_small=reshape(U_small,size(z_pic,1),size(z_pic,2));
    subplot(2,3,k)
    pcolor(flipud(U_small))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("EigenFace Mode "+ string(k))
end

f5 = figure;
ProjectionsValues = SingleFaces'*U_firstTwenty;
RankTwentyApproxImages = U_firstTwenty*ProjectionsValues';
countPlotted = 0;
for k = 1:2
    Approx = reshape(RankTwentyApproxImages(:,k),size(z_pic,1),size(z_pic,2));
    Full = reshape(SingleFaces(:,k),size(z_pic,1),size(z_pic,2));
    countPlotted = countPlotted + 1;
    subplot(2,2,countPlotted)
    pcolor(flipud(Approx))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("Rank " + string(numEigenfaces) +" approx.")
    
    countPlotted = countPlotted + 1;
    subplot(2,2,countPlotted)
    pcolor(flipud(Full))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("Full Image")    
    
end


% -----------------------------------------------------


%% Uncropped images ----------------------------------
z_pic = imresize(imread("C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces\subject01.centerlight","gif"),[120 80]);
%imshow(z_pic)


% Reading in images
count = 0;
AllImages2 = zeros(size(z_pic,1),size(z_pic,2),165);

%Read in images and save in 3-d matrix
D = dir("C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces");
for k = 3:167
    strName = D(k).name;
    count = count+1;
    currentfile = "C:\Users\Karl\Documents\Documents\AMATH584\hw2\yalefaces\" + strName;
    A = imresize(double(imread(currentfile,"gif")),[120 80]);
    AllImages2(:,:,count) = A;
end


z_A = AllImages2(:,:,1);
C_ColumnPic2 = zeros(size(z_A,1)*size(z_A,2),165);
for j = 1:165
    A = AllImages2(:,:,j);
    A = reshape(A,size(A,1)*size(A,2),1);
    C_ColumnPic2(:,j) = A;
end


% Corr_Matrix2 = C_ColumnPic2*C_ColumnPic2';


% [U2,S2,V2] = svd(Corr_Matrix2,'econ');
% [v_eigs2,d_eigs2] = eigs(Corr_Matrix2,40,'lm');


load("eigenDecomp2")
load("SVD2")
S_prime = diag(S2);
S_prime = S_prime(1:100,1);
figure(f1);
hold on
plot(S_prime,'b.');
title("Singular Values Plotted on Absolute Scale")
xlabel("Singular Value Index")
ylabel("Singular Value")
legend("Cropped","Uncropped")
figure(f2);
hold on
semilogy(S_prime,'b.');
title("Singular Values Plotted on Log Scale")
xlabel("Singular Value Index")
ylabel("Singular Value")
legend("Cropped","Uncropped")

numEigenfaces = 150;
U_firstTwenty = U2(:,1:numEigenfaces);
U_firstSix = U2(:,1:6);
f4 = figure;

for k = 1:size(U_firstSix,2)
    U_small = U_firstSix(:,k);
    U_small=reshape(U_small,size(z_pic,1),size(z_pic,2));
    subplot(2,3,k)
    pcolor(flipud(U_small))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("EigenFace Mode "+ string(k))
end

f6 = figure;
ProjectionsValues = C_ColumnPic2'*U_firstTwenty;
RankTwentyApproxImages = U_firstTwenty*ProjectionsValues';
countPlotted = 0;
for k = 1:2
    Approx = reshape(RankTwentyApproxImages(:,k),size(z_pic,1),size(z_pic,2));
    Full = reshape(C_ColumnPic2(:,k),size(z_pic,1),size(z_pic,2));
    countPlotted = countPlotted + 1;
    subplot(2,2,countPlotted)
    pcolor(flipud(Approx))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("Rank " + string(numEigenfaces) +" approx.")
    
    countPlotted = countPlotted + 1;
    subplot(2,2,countPlotted)
    pcolor(flipud(Full))
    shading interp
    colormap gray
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    title("Full Image")    
    
end
