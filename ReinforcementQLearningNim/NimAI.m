function [StateRow,move] = NimAI(state,epsilon, gaveInvalidMoveFlag, prevJ)
    load('Qtable.mat','StateLookup','MoveLookup','Q_Table'); % load Q-table and lookup tables
    stateInTableFlag = 0; % set state in table flag to false;
    
    if gaveInvalidMoveFlag == 0 % if an invalid move has not been given yet
    % Check first if the state exists in the StateLookup table ----------
    for j = 1:size(StateLookup,2) % Search across all columns of the state lookup table to see if state is in table already
        if sum((state == StateLookup(:,j)) - ones(4,1)) == 0 % if the state is in the table
            stateInTableFlag = 1; % make state in table flag true
            break; % break out of loop, note that the last j value before the break represents the column of the state in the state lookup table 
        end
    end
    
    if stateInTableFlag == 0 % if the state in table flag remains false
       j = j+1; % increment j to account for new column
       StateLookup = [StateLookup,state]; % append the newly discovered state onto the state lookup table
       Q_Table = [Q_Table; zeros(1,size(Q_Table,2))]; % add a new row in Qtable for newly discovered state
    end
    
    else
        j = prevJ; % if invalid move has already been given, just use previous j value
    end
    
    % Decide on a move to make ------------------------------
    x = rand; % generate random variable from uniform distribution to determine whether to exploit or explore
    if x < epsilon % if less than epsilon value, explore. Thus epsilon is the percentage of how often exploring is done
        y = randi(size(MoveLookup,1)); % generate random integer in the range of 1-number of possible moves
    else % if greater than epsilon, exploit.
        [value,y] = max(Q_Table(j,:)); % search for highest value in Q table row corresponding to state
    end

    move = MoveLookup(y,:); % return move
    StateRow = j;
    
    save('Qtable.mat','StateLookup','MoveLookup','Q_Table'); % save variables back to mat file.

end