''' 
CISC 339 - 201910
Luis Vivar'''
'''
Three missiones and three cannibals are on onse side of a river, and they must cross a river using a boat
which can carry at most two people, under the constraint that, at any time, missionaries cannot be
outnumbered by cannibals would eat the missionaries. The boat cannot cross the river by itself with no people on board.
'''

from search import *
from utils import *
from copy import deepcopy

initialState = (7, 2, 4, 5, 0, 6, 8, 3, 1)
goalState = (1, 2, 3, 4, 5, 6, 7, 8, 0)

class EightPuzzleProblem(Problem):
    ## Constructor
    def __init__(self, initial, goal):
        self.initial = initial
        self.goal = goal

    ## Find 0 in a Tuple
    ## We assume 9 element tuple, That 0 exists in the tuple, and that there is only one 0
    ## Returns Row Col
    def findZero(self, currentState):
        col = 0
        row = 0
        for value in currentState:
            if col == 3:
                col = 0
                row += 1
            if value == 0:
                return row, col
            col += 1

    ## Get the Available actions or 'Available Moves in the puzzle'
    ## Return a List of tuples containing the Row and Col position values of spaces that can be swapped with the 0 value
    def actions(self, state):
        row, col = self.findZero(state)

        all_actions = []

        availableMovesLeft = (row, col-1)
        all_actions.append(availableMovesLeft)
        availableMovesRight = (row, col+1)
        all_actions.append(availableMovesRight)
        availableMovesUp = (row-1, col)
        all_actions.append(availableMovesUp)
        availableMovesDown = (row+1, col)
        all_actions.append(availableMovesDown)

        valid_actions = []

        for action in all_actions:
            if int(3) not in action and int(-1) not in action:
                valid_actions.append(action)
                #all_actions.remove(action)

        return valid_actions

    ## returns next state given a state and an action
    ## We assume a single action is sent
    def result(self, state, action):
        row, col = self.findZero(state)

        newState = list(state)
        temp = newState[(action[0]*3)+action[1]]
        newState[(action[0]*3)+action[1]] = 0
        newState[(row*3)+col] = temp

        return tuple(newState)

    # returns a boolean if current state is the goal state
    def goal_test(self, state):
        return state == self.goal

    # Calculate the cost to the next node based on how many missplaced tiles 
    # the new state will have
    def path_cost(self, c, state1, action, state2):
        matches = 0
        
        for (t1,t2) in zip(state1, self.goal):
            if  t1 != t2:
                matches += 1

        return matches

    # Number of missplaced tiles in the puzzle
    def h(self, node):
        matches = 0
        
        for (t1,t2) in zip(node.state, self.goal):
            if  t1 == t2:
                matches += 1

        return 9 - matches 

    # Manhattan distance
    def h2(self, node):  
        initial_config = node.state
        manDict = 0
        for i,item in enumerate(initial_config):
            prev_row,prev_col = int(i/ 3) , i % 3
            goal_row,goal_col = int(item /3),item % 3
            manDict += abs(prev_row-goal_row) + abs(prev_col - goal_col)
        return manDict
    
def main():
    problem = EightPuzzleProblem(initialState, goalState)
    initialNode = problem.initial
    print("initialNode " + str(initialNode))    
    
    finalNode = uniform_cost_search(problem)
    print("Uniform Cost Search depth:", finalNode.depth)
    for x in finalNode.path():
        print(x)
    finalNode = greedy_best_first_graph_search(problem, lambda node: problem.h(node))
    print("\nGreedy First Search depth:", finalNode.depth)
    for x in finalNode.path():
        print(x)
    finalNode = astar_search(problem, lambda node: node.path_cost + problem.h2(node))
    print("\nA Star Search depth:", finalNode.depth)
    for x in finalNode.path():
        print(x)

main()
