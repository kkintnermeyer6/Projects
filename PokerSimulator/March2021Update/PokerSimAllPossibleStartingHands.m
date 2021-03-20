clear all; close all; clc

load('StartingHandsFile.mat');

WinTiePercentages = zeros(169,2);

% iterate through all the possible starting hands.
for j = 1:169
    [wP,tP] = PokerSimFunction(2,StartingHands(j,:));
    WinTiePercentages(j,1) = wP;
    WinTiePercentages(j,2) = tP;
end