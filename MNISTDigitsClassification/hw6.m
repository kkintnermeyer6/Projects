clear; close all; clc
%% Load training and test sets just once
% Read in Training Labels
TrainingLabels = fopen('train-labels.idx1-ubyte');
A = fread(TrainingLabels,inf,'uint8');
A_prime = A(9:end,:);
ST = fclose(TrainingLabels);

% Read in Training Images
TrainingImages = fopen('train-images.idx3-ubyte');
A2 = fread(TrainingImages,inf,'uint8');
A2_prime = A2(17:end,:);
ST2 = fclose(TrainingImages);

% Read in test labels
TestLabels = fopen('t10k-labels.idx1-ubyte');
A3 = fread(TestLabels,inf,'uint8');
A3_prime = A3(9:end,:);
ST3 = fclose(TestLabels);

% read in test images
TestImages = fopen('t10k-images.idx3-ubyte');
A4 = fread(TestImages,inf,'uint8');
A4_prime = A4(17:end,:);
ST4 = fclose(TestImages);

%% Test/debug Code
% Test Code, comment out when no longer needed ----------------------------
% % Testing reading in images and double checking against labels
% ImageIndexNum = 3410; % This input controls which image-label pair is output.
% offset = (ImageIndexNum-1)*(28*28);
% 
% % Test method 1, looping through each entry in the training data vector
% % pic_test = zeros(28,28);
% % for k = 1:28
% %    for j = 1:28
% %        pic_test(k,j) = A2_prime(offset + 28*(k-1) + j,1);
% %    end
% % end
% 
% % Test Method 2, using reshape command
% pic_test = A4_prime(offset+1:offset+28*28,1);
% pic_test = reshape(pic_test,28,28);
% pic_test = pic_test.';
% 
% % print image and corresponding label from label vector
% % pcolor(pic_test); % pcolor is broken somehow, inverts/mirrors image
% imagesc(pic_test);
% colormap gray
% A3_prime((offset/(28*28))+1)
% End test code section --------------------------------------------------


%% Create Matrices for Test and Train Data
% Create regression matrices
B_ForRegression = zeros(10,length(A_prime));
for j = 1:length(A_prime)
    if A_prime(j,1) == 0
        B_ForRegression(10,j) = 1;
    else
        B_ForRegression(A_prime(j,1),j) = 1;
    end
end

X_ForRegression = zeros(28*28,length(A_prime));
for k = 0:length(A_prime)-1
    pic_temp = A2_prime((k*28*28)+1:(k+1)*28*28,1);
    pic_temp = reshape(pic_temp,28,28);
    pic_temp = pic_temp.';
    pic_temp = reshape(pic_temp,28*28,1);
    X_ForRegression(:,k+1) = pic_temp;
end

% double check reshaping operation, comment when done -------------------
% index = 957;
% imagesc(reshape(A_ForRegression(:,index),28,28));
% colormap gray
% A_prime(index)
% end double checking --------------------------------------------------

% Create Testing Matrices
B_ForTesting = zeros(10,length(A3_prime));
for j = 1:length(A3_prime)
    if A3_prime(j,1) == 0
        B_ForTesting(10,j) = 1;
    else
        B_ForTesting(A3_prime(j,1),j) = 1;
    end
end

X_ForTesting = zeros(28*28,length(A3_prime));
for k = 0:length(A3_prime)-1
    pic_temp = A4_prime((k*28*28)+1:(k+1)*28*28,1);
    pic_temp = reshape(pic_temp,28,28);
    pic_temp = pic_temp.';
    pic_temp = reshape(pic_temp,28*28,1);
    X_ForTesting(:,k+1) = pic_temp;
end

%% Model Training
% pinv trained
A_Trained_Pinv = B_ForRegression*pinv(X_ForRegression);
B_output_TrainedPinvModel = A_Trained_Pinv*X_ForTesting;

[M,I] = max(B_output_TrainedPinvModel);
I = I.';
bool = I ~= 10;
I = I.*bool;
bool = I == A3_prime;
successes = sum(bool);
successrate = successes/10000;

%%
norms = 0*A_Trained_Pinv(1,:);
for j = 1:length(norms)
   norms(1,j) = norm(A_Trained_Pinv(:,j),2);     
end

cutoff = 10^-14;
bool2 = norms >= cutoff;
% norms = norms.*bool2;
boolTemp = bool2;
for j = 1:9
   bool2 = [bool2;boolTemp]; 
end

A_Trained_Pinv_Sparse = bool2.*A_Trained_Pinv;
B_output_TrainedPinvModel_Sparse = A_Trained_Pinv_Sparse*X_ForTesting;
[M,I] = max(B_output_TrainedPinvModel_Sparse);
I = I.';
bool = I ~= 10;
I = I.*bool;
bool = I == A3_prime;
successes = sum(bool);
successrate = successes/10000;


temp = norms.';
temp = log10(temp);
temp = reshape(temp,28,28);
imagesc(temp); colorbar

%%
% lasso trained
lambdaVals = [0.01,0.001,0.0001,0.00001];
lambdaVals = fliplr(lambdaVals);
A_Trained_Lasso = zeros(size(A_Trained_Pinv.',1),size(A_Trained_Pinv.',2),length(lambdaVals));

for k = 1:10
[A_vec_Trained_Lasso,~] = lasso(X_ForRegression.',B_ForRegression(k,:).','Lambda',lambdaVals);
A_Trained_Lasso(:,k,:) = A_vec_Trained_Lasso;

end

%%
B_output_TrainedLassoModel = zeros(size(B_output_TrainedPinvModel,1),size(B_output_TrainedPinvModel,2),4);
for k = 1:4
    B_output_TrainedLassoModel(:,:,k) = A_Trained_Lasso(:,:,k).'*X_ForTesting;
    
end

maxes_B_Lasso = zeros(4,size(B_output_TrainedLassoModel,2));
for k = 1:4
    [M,I] = max(B_output_TrainedLassoModel(:,:,k));
    maxes_B_Lasso(k,:) = I;
end
maxes_B_Lasso = maxes_B_Lasso.';
maxes_B_Lasso = maxes_B_Lasso.*(maxes_B_Lasso ~= 10);

A3_prime_expanded = [A3_prime,A3_prime,A3_prime,A3_prime];
bool = maxes_B_Lasso == A3_prime_expanded;
successrate_lasso = sum(bool);
successrate_lasso = successrate_lasso *(1/10000);

%%
A_temp = A_Trained_Lasso(:,:,4);
A_temp = A_temp.';
norms2 = zeros(1,size(A_temp,2));
for j = 1:length(norms2)
   norms2(1,j) = norm(A_temp(:,j),2); 
end

norms2 = log10(norms2);
norms2_reshaped = reshape(norms2.',28,28);
figure;
imagesc(norms2_reshaped); colorbar;

%% Repeated analysis for each digit individually
A_Trained_Pinv_Short_All = zeros(10*size(A_Trained_Pinv,1),size(A_Trained_Pinv,2));
for q = 0:9
    bool_digit = A_prime == q;
    Q = find(bool_digit);
    
    % Pull out the training data corresponding to the indices in Q here
    X_Regression_Short = zeros(784,length(Q));
    for j = 1:length(Q)
        offset = Q(j)-1;
        tempVector = A2_prime((offset*784+1):(offset*784+784),1);
        tempVector = reshape(tempVector,28,28);
        tempVector = tempVector.';
        tempVector = reshape(tempVector,784,1);
        X_Regression_Short(:,j) = tempVector;
    end
    
    B_Regression_Short = zeros(10,1);
    B_Regression_Short(q+1,1) = 1;
    B_Regression_Short = [B_Regression_Short(2:10,1);B_Regression_Short(1,1)];
    tempVector = B_Regression_Short;
    for j = 1:length(Q)-1
        B_Regression_Short = [B_Regression_Short,tempVector];
    end
    
    % Once you have that, you can repeat the lasso regression.
    % comment out lasso approach -----------------------------------------
%     lambda = 0;
%     A_Trained_Lasso_Short = zeros(10*size(A_Trained_Pinv,1),size(A_Trained_Pinv,2));
%     for k = 1:10
%         [A_vec_Trained_Lasso_Short,~] = lasso(X_Regression_Short.',B_Regression_Short(k,:).','Lambda',lambda);
%         A_vec_Trained_Lasso_Short = A_vec_Trained_Lasso_Short.';
%         A_Trained_Lasso_Short(10*q+k,:) = A_vec_Trained_Lasso_Short;
%     end
    % -------------------------------------------------------------------
    
    % lasso regression seems to not like a column vector that is all the
    % same number (in this case either 1 or 0) so try pinv() approach instead.
    A_Trained_Pinv_Short = B_Regression_Short*pinv(X_Regression_Short);
    
    A_Trained_Pinv_Short_All(10*q+1:10*q+10,:) = A_Trained_Pinv_Short;
    
    
end

A_Trained_Pinv_Short_All = [A_Trained_Pinv_Short_All(11:end,:);A_Trained_Pinv_Short_All(1:10,:)];

%%
cutoff_second = 10^-14;
bool4 = abs(A_Trained_Pinv_Short_All) >= cutoff_second;
A_Trained_Pinv_Short_All_Amended = bool4.*A_Trained_Pinv_Short_All;
A_Trained_Pinv_Short_All_Amended = A_Trained_Pinv_Short_All_Amended.';
A_Trained_Pinv_Short_All_Amended = abs(A_Trained_Pinv_Short_All_Amended);

%%
for j = 1:11:100
   temp = reshape(log10(A_Trained_Pinv_Short_All_Amended(:,j)),28,28);
   figure;
   imagesc(temp); colorbar;
end


