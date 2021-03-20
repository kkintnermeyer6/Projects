clear; close all; clc

% Create empty staring hands matrix, there are 169 possible starting hands: pairs, different cards suited,
% and different cards unsuited. So create a 169 x 2 matrix.
StartingHands = zeros(169,2);

% Create a matrix of the numbers 1-52 so I can figure out rank and suit of each number. This is just for me
% comment this part out when done
HelpingMatrix = 1:52;
HelpingMatrixNotOffset = uint8(HelpingMatrix);
HelpingMatrix = HelpingMatrix + 7*ones(1,52);
HelpingMatrix = uint8(HelpingMatrix);
HelpingMatrixRank = idivide(HelpingMatrix,4);
HelpingMatrixSuit = HelpingMatrix - 4*HelpingMatrixRank;
HelpingMatrix = [HelpingMatrixNotOffset;HelpingMatrix;HelpingMatrixRank;HelpingMatrixSuit];



% Create a rowindex variable to track which row of StartingHands you are editing
rowIndex = 1;

% Suited Cards
for j = 1:4:13*4
    for k = j+4:4:13*4
        if j ~= k
            StartingHands(rowIndex,:) = [j,k];
            rowIndex = rowIndex + 1;
        end
    end
end


% Unsuited Cards and pairs
for j = 1:4:13*4
    for k = j+1:4:13*4
        StartingHands(rowIndex,:) = [j,k];
        rowIndex = rowIndex + 1;
    end
end


% Sanity Check
StartingHandsRank = idivide(uint8(StartingHands+7*ones(size(StartingHands))),4);
StartingHandsSuit = uint8(StartingHands+7*ones(size(StartingHands))) - 4*StartingHandsRank;

% Now save the complete StartingHandsMatrix to a matfile called StartingHandsFile.mat