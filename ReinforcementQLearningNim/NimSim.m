clear all; close all; clc;
gameMode = 1; % 0 for AI training, 1 for AI as P1, 2 for AI as P2, 3 for 2 player/no AI
epsilon = 0; % factor for how much exploring is done, only should be used for training
N = 0; % will be used to create a for loop around the whole game sim, only one iteration if playing against AI, but N iterations for training the AI.
if gameMode == 0
    epsilon = 0.1;
    N = 100;
else
    epsilon = 0.001;
    N = 1;
end

P2wins = 0;

for iter = 1:N
GameStates = zeros(4,1);
MoveList = zeros(1,2);
A = [2;3;4;5];
turn = 1;
while sum(A) ~= 0
    turn = turn + 1; % increment turn count
    
    % store values of game state -----------------
    GameStates = [GameStates,A];
    
    % display useful information ------------------
    currentPlayerNum = mod(turn,2) + 1;
    disp("Player " + string(currentPlayerNum) + "'s Turn");
    disp(" ");
    disp("Current Game State:");
    A
    
    % initialize values to get inside the while loop ---
    x = [1,10];
    rowNum = x(1);
    valueNum = x(2);
    
    gaveInvalidMoveFlag = -1;
    SearchedRow = 0;
    while A(rowNum) < valueNum
    % requesting valid input from user or AI --------------
    gaveInvalidMoveFlag = gaveInvalidMoveFlag + 1;
    if gameMode == 0
        [r,x] = NimAI(A,epsilon, gaveInvalidMoveFlag, SearchedRow);
        SearchedRow = r;
    elseif gameMode == 1 && currentPlayerNum == 1
        [r,x] = NimAI(A,epsilon, gaveInvalidMoveFlag, SearchedRow);
        SearchedRow = r;
    elseif gameMode == 2 && currentPlayerNum == 2
        [r,x] = NimAI(A,epsilon, gaveInvalidMoveFlag, SearchedRow);
        SearchedRow = r;
    else
        x = input("Please input value in following notation, [rowNum,valueNum]");
    end
    
    rowNum = x(1);
    valueNum = x(2);
    end
    
    
    % attempt to perform operation on nim board -----------------
    A(rowNum) = A(rowNum) - valueNum;
    B = sort(A);
    A = B;
    
    
    
    % store move in movelist ----------------------------
    MoveList = [MoveList;x];
    

end

disp("Player " + currentPlayerNum + " Loses");

if currentPlayerNum == 1
    winningPlayerNum = 2;
    P2wins = P2wins + 1;
else
    winningPlayerNum = 1;
end

% update Qtable from move list and state list ---------
load('Qtable.mat','StateLookup','MoveLookup','Q_Table');
GameStates = GameStates(:,2:end);
MoveList = MoveList(2:end,:);
% Reward good moves
for k = winningPlayerNum:2:size(GameStates,2)
    z_state = GameStates(:,k);
    z_move = MoveList(k,:);
    for j = 1:size(StateLookup,2) % Search across all columns of the state lookup table to see if state is in table already
        if sum((z_state == StateLookup(:,j)) - ones(4,1)) == 0 % if the state is in the table
            break; % break out of loop, note that the last j value before the break represents the column of the state in the state lookup table 
        end
    end
    Q_Table_rownum = j;
    for j = 1:size(MoveLookup,1)
        if sum((z_move == MoveLookup(j,:)) - ones(1,2)) == 0
            break;
        end
    end
    Q_Table_colnum = j;
    Q_Table(Q_Table_rownum,Q_Table_colnum) = Q_Table(Q_Table_rownum,Q_Table_colnum) + 1;
end


% Penalize Bad Moves
for k = currentPlayerNum:2:size(GameStates,2)
    z_state = GameStates(:,k);
    z_move = MoveList(k,:);
    for j = 1:size(StateLookup,2) % Search across all columns of the state lookup table to see if state is in table already
        if sum((z_state == StateLookup(:,j)) - ones(4,1)) == 0 % if the state is in the table
            break; % break out of loop, note that the last j value before the break represents the column of the state in the state lookup table 
        end
    end
    Q_Table_rownum = j;
    for j = 1:size(MoveLookup,1)
        if sum((z_move == MoveLookup(j,:)) - ones(1,2)) == 0
            break;
        end
    end
    Q_Table_colnum = j;
    Q_Table(Q_Table_rownum,Q_Table_colnum) = Q_Table(Q_Table_rownum,Q_Table_colnum) - 1;
end

save('Qtable.mat','StateLookup','MoveLookup','Q_Table');


end


%P2wins
