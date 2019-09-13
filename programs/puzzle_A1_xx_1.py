
import os
import sys

# Heap PQ
# https://docs.python.org/2/library/heapq.html

# Running script: given code can be run with the command:
# python file.py, ./path/to/init_state.txt ./output/output.txt

from copy import deepcopy
from heapq import heappush, heappop

# Puzzle object implementing A* using manhattan distance as a heuristic.
class Puzzle(object):
    num_nodes_generated = 0
    max_frontier_size = 1
    expanded = None
    frontier = None
    frontier_map = None
    manhattan_dictionary = None

    SIZE = 3
    MAX_TILES = 9

    def __init__(self, init_state, goal_state):
        self.init_state = init_state # Board
        self.goal_state = goal_state
        self.actions = list() # List of possible actions

        # Set these afterwards.
        self.parent = None
        self.pathcost = 0
        self.empty_tile_position = None
        self.action = None

        # Statistics
        Puzzle.num_nodes_generated += 1

    #Override
    def __eq__(self, other):
        # Defines equality of two Puzzle objects.
        # Defined as equal if init_state is the same.
        return isinstance(other, Puzzle) and (self.init_state == other.init_state)

    def to_string(self):
        string_represntation = ""
        for i in range(3):
            for j in range(3):
                string_represntation += str(self.init_state[i][j])

        return string_represntation

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
        Puzzle.manhattan_dictionary = Puzzle.create_manhattan_dictionary()

        # Check for solvability.
        if not self.is_solvable():
            return "UNSOLVABLE"

        while Puzzle.frontier:
            # Update max frontier size.
            if (len(Puzzle.frontier) > Puzzle.max_frontier_size):
                Puzzle.max_frontier_size = len(Puzzle.frontier)

            # Remove node from frontier - frontier contains tuple of (pathcost + heuristic, PUZZLE object)
            successor = heappop(self.frontier)
            node = successor[1]

            # Check that this node is not an "old" node with worse path cost.
            if successor[0] > Puzzle.frontier_map.get(node.to_string()):
                continue
            else:
                # Remove node from frontier_map and continue with the code.
                del Puzzle.frontier_map[node.to_string()]

            # Goal test
            # node[1] is the state
            if self.is_goal(node.init_state):
                print "STATISTICS: NUM NODES GENERATED: ", Puzzle.num_nodes_generated
                print "STATISTICS: MAX FRONTIER SIZE: ", Puzzle.max_frontier_size
                print "Given start state: ", self.init_state
                print "Make sure you reached goal state. Current state: ", node.init_state
                return node.format_solution([])

            # Add to explored.
            Puzzle.expanded.add(node.to_string())
            node.generate_actions_and_states()
            node.enqueue_successors()

        return "FAILURE"

    def format_solution(self, solution):
        parent = self.parent
        while parent is not None:
            print parent.to_string()
            solution.append(parent.action)
            parent = parent.parent

        print solution[::-1][1:]
        return solution[::-1][1:]

    def is_solvable(self):
        return self.calculate_inversions() % 2 == 0

    def calculate_inversions(self):
        flat_list = [int(i) for i in list(self.to_string())]
        inversions = 0
        for i in range(Puzzle.MAX_TILES):
            for j in range(i, Puzzle.MAX_TILES):
                if flat_list[i] > flat_list[j] and flat_list[i] != 0 and flat_list[j] != 0:
                    inversions += 1

        print inversions
        return inversions

    def initialize_astar(self):
        # Init frontier (PQ), explored set, expanded STATES, soln list.
        Puzzle.frontier = [] # Use heappush(pq, value) to enter value.
        Puzzle.expanded = set() # Python set has O(1) lookup.
        Puzzle.frontier_map = {} # {State in string rep : eval fn}

        # Add initial node to the frontier
        heappush(Puzzle.frontier, (self.pathcost, self)) # Gives a tuple with the 1st being path cost, 2nd self.
        # Add to frontier set
        Puzzle.frontier_map[self.to_string()] = self.pathcost

    def set_puzzle_parameters(self, parent, pathcost, action):
        self.parent = parent
        self.pathcost = pathcost
        self.action = action


    def find_empty_tile(self):
        # Find current location (i,j) of empty tile.
        i, j = 0, 0
        for list in self.init_state:
            for value in list:
                if value == 0:
                    self.empty_tile_position = (i, j) # Set empty tile position; no need to return (i,j)
                    return
                else:
                    j= (j + 1) % self.SIZE

            i = (i + 1) % self.SIZE

    def generate_actions_and_states(self):
        # Returns list of tuples containing (action, new state)
        # All states are reachable from current state
        self.find_empty_tile()  # Location at empty_tile_position

        i, j = self.empty_tile_position
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
            successor_eval = successor_pathcost + successor.calculate_manhattan()
            successor.set_puzzle_parameters(self, successor_pathcost, action[0])

            # Check that action state is not in explored. If not in explored, check if in frontier

            if successor.to_string() not in Puzzle.expanded:
                state_current_cost = Puzzle.frontier_map.get(successor.to_string())
                # If not in frontier too, add to frontier.
                # else if in frontier AND f(n) in frontier is higher, replace that node with this successor.
                if state_current_cost is None:
                    heappush(Puzzle.frontier, (successor_eval, successor))
                    Puzzle.frontier_map[successor.to_string()] = successor_eval
                else: # State is not in expanded, is in frontier
                    if state_current_cost > successor_eval:
                        # Update frontier map, add successor to queue
                        heappush(Puzzle.frontier, (successor_eval, successor))
                        Puzzle.frontier_map[successor.to_string()] = successor_eval

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
                actual_position = Puzzle.manhattan_dictionary.get(value)
                total += abs(i - actual_position[0]) + abs(j - actual_position[1])
        return total


    def is_goal(self, state):
        return state == self.goal_state
        # GOAL STATE:
        # 123
        # 456
        # 780

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







