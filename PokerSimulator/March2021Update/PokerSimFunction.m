function [winPercent_out,tiePercent_out] = PokerSimFunction(numPlayers_in,StartingHand_in)


% This will turn on short circuiting for just checking if player 1 wins, if
% you want to see who wins each hand then set this to zero, if you just
% care about seeing how many times player 1 has won, turn this to 1.
turnOnPlayerOneWinShortCircuiting = 1;


numSims = 1000000;
% What we can do here is make a matrix that is numSims rows by
% totalCardNumber columns
% and then pregenerate all of the randperms for each game instance, each to its own
% row. Then we can run the routine to zero out any that match the preset
% hand cards all at once. This works because the preset hands and board
% cards are defined as "global" variables. I.e. they are defined outside of the
% loop that loops through game instances. However, we need to be careful
% because depending on number of simulations, we might reach a memory
% bottleneck

% Another idea we need to do is to save the values of number of sims
% and number of wins for a given predefined hand into a separate file so that you
% can rerun this script multiple times to add more total trials without losing 
% the information that was saved from previous trials of the script. right
% now you're limited to about 1,000,000 sims per time you hit "run" on
% this script. if you could save the numSims value and number of wins value
% to a separate data file, then you could hit "run" on this script 10 separate times
% and effectively increase your number of trials to 1,000,000,000 or more
% by saving all the data fro mthe previous times you hit "run"





% Code Block One ---------------------------------------------------------
numPlayers = numPlayers_in;

numPresetHands = 1;
presetHands = zeros(1,2*numPresetHands);
% insert specific cards here in a 1-52 format in a 1-52 range, offset will
% be added in the next line of code
presetHands(1,1) = StartingHand_in(1,1);
presetHands(1,2) = StartingHand_in(1,2);
presetHands = presetHands + 7*ones(1,2*numPresetHands);
presetHandsInt = uint8(presetHands);
presetHandsIntRank = idivide(presetHandsInt,4);
presetHandsIntSuit = presetHandsInt - 4*presetHandsIntRank;

numPresetBoardCards = 0;
presetBoardCards = zeros(1,numPresetBoardCards);
% insert specific cards here in a 1-52 format in a 1-52 range, offset will
% be added in the next line of code
presetBoardCards = presetBoardCards + 7*ones(1,numPresetBoardCards);
presetBoardCardsInt = uint8(presetBoardCards);
presetBoardCardsIntRank = idivide(presetBoardCardsInt,4);
presetBoardCardsIntSuit = presetBoardCardsInt - 4*presetBoardCardsIntRank;

numRandomlyGeneratedBoardCards = 5-numPresetBoardCards;

totalCardNumber = 2*numPlayers+5;

numberRandomCardsNeeded = totalCardNumber - (2*numPresetHands) - numPresetBoardCards;

ShuffledDeckAllGameInstances = zeros(numSims,totalCardNumber);

for l = 1:numSims
    ShuffledDeckAllGameInstances(l,:) = randperm(52,totalCardNumber);
end

% Offset the numbers by 7 so that the real range of numbers is 8-59
ShuffledDeckAllGameInstances = ShuffledDeckAllGameInstances + 7*ones(size(ShuffledDeckAllGameInstances));

% End Code Block One -----------------------------------------------------





% Code Block Two ---------------------------------------------------------

% Search for any 1-52 format numbers that already exist in preset hands or preset board
% cards and zero them out in the shuffled deck. Use ismember function here
% instead of for loops.
temp = [presetHands,presetBoardCards];
Lia = ismember(ShuffledDeckAllGameInstances,temp);
Lia2 = ~Lia;
ShuffledDeckAllGameInstances = ShuffledDeckAllGameInstances.*Lia2;
% obsolete -----------------------------------------------------
% for j = 1:2*numPresetHands
%     bool = ShuffledDeckAllGameInstances ~= presetHands(1,j);
%     ShuffledDeckAllGameInstances = ShuffledDeckAllGameInstances.*bool;
% end
% for k = 1:numPresetBoardCards
%     bool = ShuffledDeckAllGameInstances ~= presetBoardCards(1,k);
%     ShuffledDeckAllGameInstances = ShuffledDeckAllGameInstances.*bool;  
% end
% ---------------------------------------------------------------

% End Code Block Two -----------------------------------------------------





% Code Block Three -------------------------------------------------------

% Create a uint8 version of the shuffled Deck all game instances, taking
% into account removing any zeroed out terms
ShuffledDeckAllGameInstancesInt = zeros(numSims,numberRandomCardsNeeded);

for b = 1:numSims
% Pull off a single row of Shuffled Deck for all instances.
ShuffledDeck = ShuffledDeckAllGameInstances(b,:);

% Throw out all zero values in shuffled deck
ShuffledDeck = ShuffledDeck(ShuffledDeck~=0);

% Pick only the first m values where m is equal to numRandomCardsNeeded,
ShuffledDeck = ShuffledDeck(1,1:numberRandomCardsNeeded);

% Store the shortened version into the shuffled deck all instances uint8
% matrix
ShuffledDeckAllGameInstancesInt(b,:) = ShuffledDeck;
    
end

% Put preset hand cards and preset board cards into the correct spot
presetHandsForAllInstances = ones(numSims,length(presetHands));
presetHandsForAllInstances = presetHandsForAllInstances*diag(presetHands');

presetBoardCardsForAllInstances = ones(numSims,length(presetBoardCards));
presetBoardCardsForAllInstances = presetBoardCardsForAllInstances*diag(presetBoardCards');

ShuffledDeckAllGameInstancesInt = [presetHandsForAllInstances,ShuffledDeckAllGameInstancesInt(:,1:size(ShuffledDeckAllGameInstancesInt,2)-5+numPresetBoardCards),presetBoardCardsForAllInstances,ShuffledDeckAllGameInstancesInt(:,size(ShuffledDeckAllGameInstancesInt,2)-4+numPresetBoardCards:size(ShuffledDeckAllGameInstancesInt,2))];
% obsolete ---------------
% ShuffledDeckAllGameInstancesInt = [presetHandsForAllInstances,ShuffledDeckAllGameInstancesInt(:,1:size(ShuffledDeckAllGameInstancesInt,2)-5+numPresetBoardCards),presetBoardCardsForAllInstances,ShuffledDeckAllGameInstancesInt(:,size(ShuffledDeckAllGameInstancesInt,2)-5+numPresetBoardCards:end)];
% ------------------------

% convert the uint8 shuffled deck for all instances to a uint8 matrix
ShuffledDeckAllGameInstancesInt = uint8(ShuffledDeckAllGameInstancesInt);

% Do a integer divide by 4 and mod 4 operation to get ranks and suits
ShuffledDeckAllGameInstancesIntRank = idivide(ShuffledDeckAllGameInstancesInt,4);
ShuffledDeckAllGameInstancesIntSuit = ShuffledDeckAllGameInstancesInt-4*ShuffledDeckAllGameInstancesIntRank;

% End Code Block Three ---------------------------------------------------





% Now that we have all Ranks and suits, we can proceed to score all the
% hands row by row. while we score hands, we can also return values for a
% tiebreak. To do this, when you score a hand you return a row vector of
% values, the first entry is the hand rank, the second entry is the value
% of the first tiebreak rank, third value is the second tiebreak rank, etc.
% so for exampe, a full house of three aces and 2 eights would return [7,
% 14, 8] the 7 is the full house ranking, the 14 is the ranking of the
% three of a kind (in this case aces) and the 8 is the ranking of the pair
% (in this case 8s). This can be used to quickly compare hands of similar
% rank.

% Another idea to speed up the process of per-hand scoring is have a running value
% that represents the current highest scoring hand and to skip all further
% hand scorings that do not at least meet that minimum rank. for example,
% if player 1 has a four-of-a-kind, then you pass "four-of-a-kind" as the current highest hand and
% when you score player 2s hand, if
% he does not have at least a four of a kind, you can skip checking for two
% pair, three of a kind, etc. because he wil never beat the four of a kind
% hand if he does not also have at minimm a four of a kind.

% Clear all variables tht are not ShuffledDeckAllGameInstancesIntRank or
% ShuffledDeckAllGameInstancesIntSuit here, all of the other matrices that
% were created were just to help set up the shuffled deck, since we now
% have the cards in rank-suit format, we can clear all the other matrices
% to save memory space.





% Code Block Four ---------------------------------------------------------

% Begin Scoring Hands
numPlayerOneWins = 0;
numPlayerOneTies = 0;
for b = 1:numSims
    % We are now looking at a single game instance
   ScoredHands = zeros(numPlayers,6);
   % HighestHandRank = 1;
   HighestHandRank = 0;
   continuedHighestHandRank = 0;
   numTimesHighestHandRankIsReassigned = 0;
   
   % Looking at each player's hand and scoring it.
   for k = 1:numPlayers
       Hand_top = [ShuffledDeckAllGameInstancesIntRank(b,(2*(k-1)+1):(2*(k-1)+2)),ShuffledDeckAllGameInstancesIntRank(b,(end-4):end)];
       Hand_bottom = [ShuffledDeckAllGameInstancesIntSuit(b,(2*(k-1)+1):(2*(k-1)+2)),ShuffledDeckAllGameInstancesIntSuit(b,(end-4):end)];
       Hand = [Hand_top;Hand_bottom];
       % obsolete ----------------
       % Hand = [ShuffledDeckAllGameInstancesIntRank(2*(k-1)+1:2*(k-1)+2,:),ShuffledDeckAllGameInstancesIntRank(end-5:end,:);ShuffledDeckAllGameInstancesIntSuit(2*(k-1)+1:2*(k-1)+2,:),ShuffledDeckAllGameInstancesIntSuit(end-5:end,:)];
       % -------------------------
       [MyScoredHand,continuedHighestHandRank] = ScoreThatPokerHand(Hand,HighestHandRank);
       
       % counting how many times the HighestHandRank is reassigned, if it's
       % reassigned more than once, we know that player 1 has lost, this is
       % also controlled by the turnOnPlayerOneWinShortCircuiting variable
       % which will turn this functionality on or off as desired by the
       % user
       if  turnOnPlayerOneWinShortCircuiting && (continuedHighestHandRank ~= HighestHandRank)
           numTimesHighestHandRankIsReassigned = numTimesHighestHandRankIsReassigned + 1;
       end
       
       HighestHandRank = continuedHighestHandRank;
       ScoredHands(k,:) = MyScoredHand;
       
       % if HighestHandRank has been reassigned more than once, player 1
       % has lost and there's no reason to keep going through the loop,
       % this is also controlled by the turnOnPlayerOneWinShortCircuiting
       % variable which will turn this functionality on or off as desired
       % by the user
       if turnOnPlayerOneWinShortCircuiting && (numTimesHighestHandRankIsReassigned > 1)
           break;
       end
       
   end
   
   if numTimesHighestHandRankIsReassigned <= 1
    % figure out winner of game instance here. We can use a little trick
    % here. Since we want to go column by column through the ScoredHands
    % matrix until we find one player that has the highest scoring hand, we
    % can associate a power of 10 with each column and sum the columns so
    % that the player with the highest hand will have the highest sum. The
    % reason this works is because we first look to the first column to see
    % if there is only one player with the highest hand, if so, the 10^5
    % term will dominate anything else, if there are more than one players,
    % then we move to the 10^4 term and so on until there's only one
    % winner.
    comparePlayers = ScoredHands*[10^10,10^8,10^6,10^4,10^2,1]';
    [M,I] = max(comparePlayers);
    
    % max commmand only returns the first index of the maximum value, we need to
    % double check for cases where there is a tie
    if sum(comparePlayers == M) > 1
        % there's a tie/split pot
        numPlayerOneTies = numPlayerOneTies + 1;
    end
    
    if I == 1
        numPlayerOneWins = numPlayerOneWins + 1; % The way this is defined means that it also increments on a tie, 
        % we'll remove those double-counts at the end
    end
    
   end
   
end

% Correct for double-counting ties
numPlayerOneWins = numPlayerOneWins - numPlayerOneTies;

% Calculate win percents and tie percents
winPercent_out = numPlayerOneWins/numSims;
tiePercent_out = numPlayerOneTies/numSims;


% End Code Block Four -----------------------------------------------------





% Below here is obsolete, comment out when done ------------------------------------------
% for a = 1:numSims
% % Now we are inside a single game instance loop
% 
% % Pull off a single row of Shuffled Deck for all instances.
% ShuffledDeck = ShuffledDeckAllGameInstances(a,:);
% 
% % Throw out all zero values in shuffled deck
% ShuffledDeck = ShuffledDeck(ShuffledDeck~=0);
% 
% % Pick only the first m values where m is equal to numRandomCardsNeeded,
% ShuffledDeck = ShuffledDeck(1,1:numberRandomCardsNeeded);
% ShuffledDeckInt = uint8(ShuffledDeck);
% ShuffledDeckIntRank = idivide(ShuffledDeckInt,4);
% ShuffledDeckIntSuit = ShuffledDeckInt - 4*ShuffledDeckIntRank;
% 
% % Note that cards in the 1-52 number format will be spread across three separate Matrices: presetHands,
% % presetBoardCards, and ShuffledDeck to avoid changing matrix size every
% % time through the loop. This is to make the sim go faster.
% 
% % Put all of the 1-52 format cards into a two row format using integer
% % divide by 4 and mod 4 operations
% twoRowCardMatrix = zeros(2:totalCardNumber);
% 
% % put in preset hands first
% twoRowCardMatrix(1,1:2*numPresetHands) = presetHandsIntRank;
% twoRowCardMatrix(2,1:2*numPresetHands) = presetHandsIntSuit;
% 
% % then put in the rest of the randomly generated cards that are NOT board
% % cards
% twoRowCardMatrix(1,2*numPresetHands+1:end-5) = ShuffledDeckIntRank(1,1:end-numRandomlyGeneratedBoardCards);
% twoRowCardMatrix(2,2*numPresetHands+1:end-5) = ShuffledDeckIntSuit(1,1:end-numRandomlyGeneratedBoardCards);
% 
% % then put in the preset board cards
% twoRowCardMatrix(1,end-4:end-numRandomlyGeneratedBoardCards) = presetBoardCardsIntRank;
% twoRowCardMatrix(2,end-4:end-numRandomlyGeneratedBoardCards) = presetBoardCardsIntSuit;
% 
% % then put in the rest of the randomly generated board cards
% twoRowCardMatrix(1,end-numRandomlyGeneratedBoardCards+1:end) = ShuffledDeckIntRank(1,end-numRandomlyGeneratedBoardCards+1:end);
% twoRowCardMatrix(2,end-numRandomlyGeneratedBoardCards+1:end) = ShuffledDeckIntSuit(1,end-numRandomlyGeneratedBoardCards+1:end);
% 
% % Now our matrix that stores all the the cards is ready to be thrown into
% % the scoring system.
% 
% 
% end
% -----------------------------------------------------------------------




end