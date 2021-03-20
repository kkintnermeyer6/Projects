function [MySillyScoredHand,continuingHighestHand] = ScoreThatPokerHand(Hand,currentHighestHand)
    MySillyScoredHand = zeros(1,6);
    continuingHighestHand = currentHighestHand;

    % Check for Straight Flush, first check for flush, then for straight
    % Straight-Flush Flush Check
    FlushFlag = 0;
    for k = 0:3
        boolFlush = Hand(2,:) == k;
        if sum(boolFlush) >= 5
            FlushFlag = 1;
            break;
        end
    end
    
    % Straight-Flush Straight Check
    if FlushFlag
        flushCards = Hand(1,:);
        flushCards = flushCards(Hand(2,:)==k); % this might break, double check this
        sortedFlushCards = sort(flushCards);
        if sum(sortedFlushCards == 14) >= 1 % account for ace high or low
            sortedFlushCards = [1,sortedFlushCards];
        end
        for l = length(sortedFlushCards)-4:-1:1
            if (sortedFlushCards(l+1) == sortedFlushCards(l)+1) && (sortedFlushCards(l+2) == sortedFlushCards(l)+2) && (sortedFlushCards(l+3) == sortedFlushCards(l)+3) && (sortedFlushCards(l+4) == sortedFlushCards(l)+4)
                MySillyScoredHand(1,1) = 9;
                continuingHighestHand = 9;
                MySillyScoredHand(1,2:6) = fliplr(sortedFlushCards(1,l:l+4));
                return;
            end 
        end
    end
    
    if 8 >= currentHighestHand
    % Check for Four-of-a-kind
    NumCardsOfEachRank = zeros(1,13);
    for k = 1:size(Hand,2)
        NumCardsOfEachRank(1,Hand(1,k)-1) = NumCardsOfEachRank(1,Hand(1,k)-1) + 1;
    end
    
    % Obsolete method for populating NumCardsOfEachRank. Took too many operations ---------
%     for k = 2:14
%         bool = Hand(1,:) == k;
%         NumCardsOfEachRank(1,k-1) = sum(bool);
%     end
    % ------------------------------------------------------------------------------------
    
    bool2 = NumCardsOfEachRank == 4;
    if sum(bool2) > 0
        % Quad rank is 8 (second highest hand)
        MySillyScoredHand(1,1) = 8;
        continuingHighestHand = 8;
        
        % Find rank of card for which the Quad exists
        index = find(bool2);
        MySillyScoredHand(1,2) = index;
        
        % Find the remaining highest card that is not included in the quad
        % bool3 = (NumCardsOfEachRank ~= 4) && (NumCardsOfEachRank ~= 0);
        bool3 = NumCardsOfEachRank ~=4 & NumCardsOfEachRank ~=0;
        indeces = find(bool3);
        MySillyScoredHand(1,3) = indeces(1,end); % Need to double check that this returns highest index that meets criteria
        
        return;
    end
    
    % obsolete, comment out --------------------------------------------
%     PairsTripsAndQuads = zeros(1,3);
%     for k = 2:4
%         bool2 = NumCardsOfEachRank == k;
%         PairsTripsAndQuads(1,k-1) = sum(bool2);
%     end
%     
%     if PairsTripsAndQuads(1,3) > 0
%        MySillyScoredHand(1,1) = 8;
%        % Save Tiebreak Ranking Stuff
%        
%        return;
%     end
    % -----------------------------------------------------------------
    end
    
    if 7 >= currentHighestHand
    % Check for Full House
    boolTrips = NumCardsOfEachRank == 3;
    boolPairs = NumCardsOfEachRank == 2;
    
    if (sum(boolTrips) >= 2) || (sum(boolTrips) >= 1 && sum(boolPairs) >= 1)
       % There's a full house here.
       MySillyScoredHand(1,1) = 7;
       continuingHighestHand = 7;
       
       % Tiebreak stuff,find highest trip first
       indeces = find(boolTrips);
       MySillyScoredHand(1,2) = indeces(1,end);
       
       % Then find highest pair or lower trip
       if sum(boolTrips) >= 2
           MySillyScoredHand(1,3) = indeces(1,end-1);
       else
           indeces = find(boolPairs);
           MySillyScoredHand(1,3) = indeces(1,end);
       end
       
       return;
    end
    end
    
    if 6 >= currentHighestHand
    % Check for Flush
    if FlushFlag
        MySillyScoredHand(1,1) = 6;
        continuingHighestHand = 6;
        flippedSortedFlushCards = fliplr(sortedFlushCards);
        MySillyScoredHand(1,2:end) = flippedSortedFlushCards(1:5);
        return;
    end
    end
    
    if 5 >= currentHighestHand
    % Check for Straight
    handRank = Hand(1,:);
    handRank = sort(handRank);
    if sum(handRank == 14) >= 1 % account for ace high or low
        handRank = [1,handRank];
    end
    for l = length(handRank)-4:-1:1
        if (handRank(l+1) == handRank(l)+1) && (handRank(l+2) == handRank(l)+2) && (handRank(l+3) == handRank(l)+3) && (handRank(l+4) == handRank(l)+4)
            MySillyScoredHand(1,1) = 5;
            continuingHighestHand = 5;
            MySillyScoredHand(1,2:6) = fliplr(handRank(1,l:l+4));
            return;
        end 
    end
    end
    
    if 4 >= currentHighestHand
    % Check for Three-of-a-kind
    % bool4 = NumCardsOfEachRank == 1;
    boolSingles = NumCardsOfEachRank == 1;
    if sum(boolTrips) >= 1
        MySillyScoredHand(1,1) = 4;
        continuingHighestHand = 4;
        MySillyScoredHand(1,2) = find(boolTrips);
        indeces = find(boolSingles);
        indecesFlipped = fliplr(indeces);
        MySillyScoredHand(1,3:4) = indecesFlipped(1,1:2);
        return;
    end
    end
    
    if 3 >= currentHighestHand
    % Check for Two-pair
    if sum(boolPairs) >= 2
        MySillyScoredHand(1,1) = 3;
        continuingHighestHand = 3;
        indeces = find(boolPairs);
        indecesFlipped = fliplr(indeces);
        MySillyScoredHand(1,2:3) = indecesFlipped(1,1:2);
   
        indeces = find(boolSingles);
        MySillyScoredHand(1,4) = indeces(1,end);
        return;        
    end
    end
    
    if 2 >= currentHighestHand
    % Check for pair
    if sum(boolPairs) >= 1
        MySillyScoredHand(1,1) = 2;
        continuingHighestHand = 2;
        indeces = find(boolPairs);
        MySillyScoredHand(1,2) = indeces(1,end);
   
        indeces = find(boolSingles);
        indecesFlipped = fliplr(indeces);
        MySillyScoredHand(1,3:5) = indecesFlipped(1,1:3);
        return;                
    end
    end
    
    if 1 >= currentHighestHand
    % Check For High Card
    MySillyScoredHand(1,1) = 1;
    continuingHighestHand = 1;
    indeces = find(boolSingles);
    indecesFlipped = fliplr(indeces);
    MySillyScoredHand(1,2:6) = indecesFlipped(1,1:5);
    return;   
    end

end