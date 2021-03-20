March 2021 updates:

- Turned "PokerSim" into "PokerSimFunction", which is now a MATLAB function that returns a win percentage and a tie percentage for an input starting hand against "n" players
- Changed/Added Code in "PokerSimFunction" to track true ties when scoring a hand. 
- Wrote "GenerateAllStartingHands" to generate a list of all the possible starting hands. The list is saved in "StartingHandsFile.mat" 
- Wrote "PokerSimAllPossibleStartingHands" to call the "PokerSimFunction" for every possible starting hand. All of the win and tie percentages for each starting hand is saved in "WinTiePercentagesFile"
- Wrote "ProcessStats" to take the win/tie percentages and the list of starting hands and rank each starting hand by win percentage, outputting into a format which is more easily readable.
- ScoreThatPokerHand had no changes, it was just copied and pasted into the new subdirectory


