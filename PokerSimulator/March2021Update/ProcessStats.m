clear all; close all; clc

% Load Relevant Variables
load('StartingHandsFile.mat');
load('WinTiePercentagesFile.mat');

% Convert StartingHands Matrix to Ranks and Suits
StartingHandsRank = idivide(uint8(StartingHands+7*ones(size(StartingHands))),4);
StartingHandsSuit = uint8(StartingHands+7*ones(size(StartingHands))) - 4*StartingHandsRank;

% Create Strings for each starting hand
StartingHandsString = string(StartingHandsRank);
StartingHandsString = strrep(StartingHandsString,'14','A');
StartingHandsString = strrep(StartingHandsString,'13','K');
StartingHandsString = strrep(StartingHandsString,'12','Q');
StartingHandsString = strrep(StartingHandsString,'11','J');
StartingHandsString = strrep(StartingHandsString,'10','T');

StartingHandsStringNew = [];
temp = 's';
for j = 1:169
    if j <= 78
        StartingHandsStringNew = [StartingHandsStringNew;StartingHandsString(j,1)+StartingHandsString(j,2)+temp];
    else
        StartingHandsStringNew = [StartingHandsStringNew;StartingHandsString(j,1)+StartingHandsString(j,2)];
    end
end


[B,I] = sort(WinTiePercentages(:,1));
B = flipud(B);
I = flipud(I);
StartingHandsWinRateRanked = StartingHandsStringNew(I);
