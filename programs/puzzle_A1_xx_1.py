
import os
import sys

# Heap PQ
# https://docs.python.org/2/library/heapq.html

#Running script takes in python file.py, init state, output action sequence.

from copy import deepcopy
from heapq import heappush, heappop


class Puzzle(object):
    num_nodes_generated = 0
    max_frontier_size = 1
    frontier = None
    SIZE = 3

    def __init__(self, init_state, goal_state):
        # You may add more attributes as necessary
        self.init_state = init_state # Board
        self.goal_state = goal_state
        self.actions = list() # List of possible actions
        self.solvable = True

        # Set these afterwards.
        self.parent = None
        self.pathcost = 0
        self.expanded_states = None
        self.solution = None
        self.empty_tile_position = None
        self.manhattan_dictionary = None
        self.action = None

        # Statistics
        self.num_nodes_generated += 1

    #Override
    def __eq__(self, other):
        # Defines equality of two Puzzle objects.
        # Defined as equal if init_state is the same.
        return isinstance(other, Puzzle) and (self.init_state == other.init_state)

    def solve(self):
        # TODO: Write your code here
        # frontier = list()

        # return: a list of actions like: ["UP", "DOWN"]
        return self.astar_search()

    # You may add more (helper) methods if necessary.
    # Note that our evaluation scripts only call the solve method.
    # Any other methods that you write should be used within the solve() method.

    def astar_search(self):
        # Return soln, unsolvable, or failure..etc.
        # Initialize
        self.initialize_astar()
        self.manhattan_dictionary = Puzzle.create_manhattan_dictionary()

        # Check for solvability.
        # if not is_solvable():
        #     return "UNSOLVABLE"
        # Check for exit conditions.
        stop = 0
        while (len(self.frontier) != 0 ):
            stop += 1
            # Update max frontier size.
            if (len(self.frontier) > self.max_frontier_size):
                self.max_frontier_size = len(self.frontier)
            # Remove node from frontier - frontier contains tuple of (pathcost + heuristic, PUZZLE object)
            successor = heappop(self.frontier)
            print "EVAL:"
            print successor[0]
            node = successor[1]
            node.solution.append(node.action)

            print "solution ", node.solution
            # Goal test
            # node[1] is the state
            if self.is_goal(node.init_state):
                print node.init_state
                return node.solution[1:] # Remove None due to initial move
            # Find empty tiles
            # Generate actions & successor states
            node.generate_actions_and_states()
            # Calculate eval fn and enqueue
            node.enqueue_successors()

        return "FAILURE"



    def initialize_astar(self):
        # Init frontier (PQ), expanded STATES, soln list.
        Puzzle.frontier = [] # Use heappush(pq, value) to enter value.
        self.expanded_states = []
        self.solution = []
        heappush(Puzzle.frontier, (self.pathcost, self)) # Gives a tuple with the 1st being path cost, 2nd self.

    def set_puzzle_parameters(self, parent, pathcost, expanded_states, solution, manhattan, action):
        self.parent = parent
        self.pathcost = pathcost
        self.expanded_states = expanded_states
        self.solution = solution
        self.manhattan_dictionary = manhattan
        self.action = action


    def find_empty_tile(self):
        # Find current location (i,j) of empty tile.
        i, j = 0, 0
        for list in self.init_state:
            for value in list:
                if value == 0:
                    self.empty_tile_position = (i, j)
                    return
                else:
                    j= (j + 1) % self.SIZE

            i = (i + 1) % self.SIZE

    def generate_actions_and_states(self):
        # Returns list of tuples containing (action, new state)
        # All states are reachable from current state
        self.find_empty_tile()  # Location at empty_tile_position

        i, j = self.empty_tile_position
        print self.init_state
        print i, j
        init_state_copy = deepcopy(self.init_state)

        # View: other tiles move towards the empty tile.
        move_up = i + 1 # J unchanged
        move_down = i - 1
        move_right = j - 1 # i unchanged
        move_left = j + 1

        if i > 0:
            self.actions.append(("DOWN", self.vertical_slide(init_state_copy, i, move_down, j)))
            #self.vertical_slide(init_state_copy, i, move_down, j) # Return to init state
        if i < self.SIZE - 1:
            self.actions.append(("UP", self.vertical_slide(init_state_copy, i, move_up, j)))
            #self.vertical_slide(init_state_copy, i, move_up, j)  # Return to init state
        if j < self.SIZE - 1:
            self.actions.append(("LEFT", self.horizontal_slide(init_state_copy, j, move_left, i)))
            #self.horizontal_slide(init_state_copy, j, move_left, i)
        if j > 0:
            self.actions.append(('RIGHT', self.horizontal_slide(init_state_copy, j, move_right, i)))

    def vertical_slide(self, orig_state, original_i, new_i, j):
        state = deepcopy(orig_state)
        temp = state[original_i][j]
        state[original_i][j] = state[new_i][j]
        state[new_i][j] = temp
        return state

    def horizontal_slide(self, orig_state, original_j, new_j, i):
        state = deepcopy(orig_state)
        temp = state[i][original_j]
        state[i][original_j] = state[i][new_j]
        state[i][new_j] = temp
        return state

    def enqueue_successors(self):

        for action in self.actions:

            # Action is (move name, state matrix)
            successor = Puzzle(action[1], self.goal_state)
            successor_pathcost = self.pathcost + 1

            successor.set_puzzle_parameters(self, successor_pathcost, self.expanded_states,
                                            self.solution, self.manhattan_dictionary, action[0])

            print action
            print "eval ", successor_pathcost + successor.calculate_manhattan()
            heappush(Puzzle.frontier, (successor_pathcost + successor.calculate_manhattan(), successor))

    @staticmethod
    def create_manhattan_dictionary():
        dict = {}
        num = 1
        for i in range(3):
            for j in range(3):
                dict[num] = (i,j)
                if (num >= 8):
                    num = 0
                else:
                    num = num + 1

        return dict

    def calculate_manhattan(self):
        total = 0
        for i in range(3):
            for j in range(3):
                value = self.init_state[i][j]
                if value == 0:
                    continue
                actual_position = self.manhattan_dictionary.get(value)
                total += abs(i - actual_position[0]) + abs(j - actual_position[1])
        return total


    def is_goal(self, state):
        return state == self.goal_state
        # ********************************#
        #                +---+---+---+   #
        #                | 1 | 2 | 3 |   #
        #                +---+---+---+   #
        #   GOAL STATE:  | 4 | 5 | 6 |   #
        #                +---+---+---+   #
        #                | 7 | 8 |   |   #
        #                +---+---+---+   #
        # ********************************#


# Java...
# /**
# 	 * Returns the number of inversions within a given puzzle.
# 	 *
# 	 * @param puzzle puzzle to returns the number of inversions for
# 	 *
# 	 * @return the number of inversions in that puzzle
# 	 */
# 	private static int getNumInversions(byte[] puzzle) {
# 		int inversions = 0;
# 		for (int i = 0; i < puzzle.length; i++) {
# 			if (puzzle[i] == 0) {
# 				continue;
# 			}
#
# 			for (int j = i; j < puzzle.length; j++) {
# 				if (puzzle[j] == 0) {
# 					continue;
# 				}
#
# 				if (puzzle[j] < puzzle[i]) {
# 					inversions++;
# 				}
# 			}
# 		}
#
# 		return inversions;
# 	}
#
# 	/**
# 	 * Returns whether or not a given puzzle is solvable.
# 	 *
# 	 * @param puzzle the puzzle to check
# 	 *
# 	 * @return {@code true} if the puzzle is solvable, otherwise {@code false}
# 	 */
# 	private static boolean isSolvable(byte[] puzzle) {
# 		return (getNumInversions(puzzle)&1) == 0;
# 	}
#
# # End  java.

if __name__ == "__main__":
    # do NOT modify below
    if len(sys.argv) != 3:
        raise ValueError("Wrong number of arguments!")

    try:
        f = open(sys.argv[1], 'r')
    except IOError:
        raise IOError("Input file not found!")

    init_state = [[0 for i in range(3)] for j in range(3)]
    goal_state = [[0 for i in range(3)] for j in range(3)]
    lines = f.readlines()

    i,j = 0, 0
    for line in lines:
        for number in line:
            if '0'<= number <= '8':
                init_state[i][j] = int(number)
                j += 1
                if j == 3:
                    i += 1
                    j = 0

    for i in range(1, 9):
        goal_state[(i-1)//3][(i-1)%3] = i
    goal_state[2][2] = 0

    puzzle = Puzzle(init_state, goal_state)
    ans = puzzle.solve()

    with open(sys.argv[2], 'a') as f:
        for answer in ans:
            f.write(answer+'\n')







