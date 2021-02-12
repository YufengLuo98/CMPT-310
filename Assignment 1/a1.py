# a1.py

from search import *
from random import shuffle
import time
import sys


# EightPuzzle Class
class EightPuzzle(Problem):
    """ The problem of sliding tiles numbered from 1 to 8 on a 3x3 board, where one of the
    squares is a blank. A state is represented as a tuple of length 9, where  element at
    index i represents the tile number  at index i (0 if it's an empty square) """

    def __init__(self, initial, goal=(1, 2, 3, 4, 5, 6, 7, 8, 0)):
        """ Define goal state and initialize a problem """
        super().__init__(initial, goal)

    def find_blank_square(self, state):
        """Return the index of the blank square in a given state"""

        return state.index(0)

    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment """

        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        index_blank_square = self.find_blank_square(state)

        if index_blank_square % 3 == 0:
            possible_actions.remove('LEFT')
        if index_blank_square < 3:
            possible_actions.remove('UP')
        if index_blank_square % 3 == 2:
            possible_actions.remove('RIGHT')
        if index_blank_square > 5:
            possible_actions.remove('DOWN')

        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """

        # blank is the index of the blank square
        blank = self.find_blank_square(state)
        new_state = list(state)

        delta = {'UP': -3, 'DOWN': 3, 'LEFT': -1, 'RIGHT': 1}
        neighbor = blank + delta[action]
        new_state[blank], new_state[neighbor] = new_state[neighbor], new_state[blank]

        return tuple(new_state)

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """

        return state == self.goal

    def check_solvability(self, state):
        """ Checks if the given state is solvable """

        inversion = 0
        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if (state[i] > state[j]) and state[i] != 0 and state[j] != 0:
                    inversion += 1

        return inversion % 2 == 0

    def h(self, node):
        """ Return the heuristic value for a given state. Default heuristic function used is
        h(n) = number of misplaced tiles """

        return sum(s != g for (s, g) in zip(node.state, self.goal))

    # Adapted search.py
    def manhattan(self, node):
        state = node.state
        index_goal = {0: [2, 2], 1: [0, 0], 2: [0, 1], 3: [0, 2],
                      4: [1, 0], 5: [1, 1], 6: [1, 2], 7: [2, 0], 8: [2, 1]}
        index_state = {}
        index = [[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2], [2, 0], [2, 1], [2, 2]]

        for i in range(len(state)):
            index_state[state[i]] = index[i]

        mhd = 0

        for i in range(9):
            for j in range(2):
                mhd = abs(index_goal[i][j] - index_state[i][j]) + mhd

        return mhd

    def gaschnig(self, node):
        moves = 0

        initial_state = node.state

        goal_state = self.goal

        while (self.goal_test(initial_state) != True):

            initial_state_list = list(initial_state)
            goal_state_list = list(goal_state)
            # find blank tile
            # while the passed in puzzle is not in goal state
            # if blank tile is not in its goal state, swap with the tile that is supposed to be at its index
            # if the blank tile is in goal state, swap it with any tile not in its goal state

            zeroIndex = self.find_blank_square(initial_state_list)
            if goal_state[zeroIndex] != 0:
                mismatchNodeGoalValue = goal_state[zeroIndex]  # Get index of 0 in goal list
                # find value where the 0 is in initial list
                mismatchNodeInitialIndex = initial_state_list.index(mismatchNodeGoalValue)
                # swap node indexes
                initial_state_list[mismatchNodeInitialIndex], initial_state_list[
                    zeroIndex] = initial_state_list[zeroIndex], initial_state_list[mismatchNodeInitialIndex]
            else:
                # find a random mismatched node to swap with
                for i in range(9):
                    if (goal_state_list[i] != initial_state_list[i]):
                        initial_state_list[i], initial_state_list[zeroIndex] = initial_state_list[zeroIndex], initial_state_list[i]
                        break
            moves += 1
            initial_state = tuple(initial_state_list)

        return moves


def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

# Search function used by astar_search


def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    start_time = time.time()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            if display:
                elapsed_time=time.time() - start_time
                print("Total Running Time:", elapsed_time, "seconds")
                print("Solution Length:", node.path_cost)
                print("Nodes expanded during search:", len(explored))
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)

    return None


###make_rand_8puzzle function###
# Create a new random EightPuzzle whenever called, if the puzzle is not solveable
# script will finish so that display is not called on an empty object
def make_rand_8puzzle():

    # Create a new EightPuzzle problem from (0,1,2,3,4,5,6,7,8)
    numbers=[0, 1, 2, 3, 4, 5, 6, 7, 8]
    # starting from index 0, swap current index with a number index from 0-8 that is not itself
    # just use shuffle function lol
    random.shuffle(numbers)

    # create a tuple of the randomized numbers list
    puzzleNumbers=tuple(numbers)

    # instantiate problem class to be used
    # problem = Problem(puzzleNumbers)

    # pass problem into EightPuzzle to initialize object
    eightPuzzle=EightPuzzle(puzzleNumbers)

    # Finally check that the puzzle is solveable,
    # if it is not solveable, print out error message,
    # if it is solveable return the puzzle object
    if (eightPuzzle.check_solvability(eightPuzzle.initial) == 0):
        print('Not solveable')
    else:
        eightPuzzle=EightPuzzle(puzzleNumbers)
        return eightPuzzle


###display FUNCTION###
# Function to display eight puzzle in a neat way with the 0 as a *
# Currently hardcoded, not optimized##
def display(state):
    puzzleDisplay=list(state)
    for x in range(9):
        # Iterate through and find the 0, replace with *
        if (puzzleDisplay[x] == 0):
            puzzleDisplay[x]='*'
    # Issue: When displaying puzzleDisplay in for loop, the 0 does display as a *
    print(puzzleDisplay[0], puzzleDisplay[1], puzzleDisplay[2])
    print(puzzleDisplay[3], puzzleDisplay[4], puzzleDisplay[5])
    print(puzzleDisplay[6], puzzleDisplay[7], puzzleDisplay[8])


if __name__ == "__main__":
    if len(sys.argv) > 1:
        listNumbers = map(int, sys.argv[1:])
        tupleNumbers=tuple(listNumbers)
        newPuzzle=EightPuzzle(tupleNumbers)
        print('-----MISMATCH-----')
        result=astar_search(newPuzzle, None, True)
        print('-----MANHATTAN-----')
        resultM = astar_search(newPuzzle, newPuzzle.manhattan, True)
        print('-----GASCHNIG-----')
        resultG = astar_search(newPuzzle, newPuzzle.gaschnig, True)
    else:
        # This takes too long...
        for i in range(10):
            newPuzzle = make_rand_8puzzle()
            if(newPuzzle):
                display(newPuzzle.initial)
                print('-----MISMATCH-----')
                result = astar_search(newPuzzle, None, True)
                print('-----MANHATTAN-----')
                resultM = astar_search(newPuzzle, newPuzzle.manhattan, True)
                print('-----GASCHNIG-----')
                resultG = astar_search(newPuzzle, newPuzzle.gaschnig, True)
            else:
                newPuzzle = make_rand_8puzzle()
