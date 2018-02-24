# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    tests:
    python pacman.py -l tinyMaze -p SearchAgent
    python pacman.py -l mediumMaze -p SearchAgent
    python pacman.py -l bigMaze -z .5 -p SearchAgent
    """
    front = util.Stack()

    # initial state has no action (the steps needed to the current state)
    front.push([[problem.getStartState()], []])
    # front_states = [problem.getStartState()]

    while not front.isEmpty():
        curr = front.pop()
        curr_state = curr[0][-1]
        # front_states.remove(curr_state)

        if problem.isGoalState(curr_state):
            return curr[1]

        for succ in problem.getSuccessors(curr_state):
            state = succ[0]
            actions = curr[1][:]

            # if the state is not already explored (path checking)
            if state not in curr[0]:  # and state not in front_states:
                new = curr[0] + [state]
                actions.append(succ[1])
                item = [new, actions]
                # print("item: ", item)
                front.push(item)
                # front_states.append(state)

    # if there is no solution
    return False


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    tests
    python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
    python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5

    for Corners:
    python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
    python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
    """
    front = util.Queue()

    # initial state has no action (the steps needed to the current state)
    front.push([problem.getStartState(), [], 0])
    front_states = [problem.getStartState()]

    # keep track all the states that has been explored
    seen = {}

    while not front.isEmpty():
        curr = front.pop()
        curr_state = curr[0]
        cost = curr[2]
        front_states.remove(curr_state)
        # print(curr)

        if problem.isGoalState(curr_state):
            return curr[1]

        seen[curr_state] = cost

        for succ in problem.getSuccessors(curr_state):
            state = succ[0]
            actions = curr[1][:]
            # update the new cost for the next step
            new_cost = cost + succ[2]

            # if the state is not already explored (cycle checking)
            if (state not in seen or new_cost < seen[state]) and state not in front_states:
                actions.append(succ[1])
                item = [state, actions, new_cost]
                # print("item: ", item)
                front.push(item)
                front_states.append(state)
                # seen[state] = new_cost

    # if there is no solution
    return False


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.

    test
    python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
    python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
    python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
    """
    front = util.PriorityQueue()

    # initial state has no action (the steps needed to the current state)
    front.push([problem.getStartState(), [], 0], 0)

    # keep track all the states that has been explored
    seen = {}

    while not front.isEmpty():
        curr = front.pop()
        curr_state = curr[0]
        cost = curr[2]
        # print(curr)

        if problem.isGoalState(curr_state):
            return curr[1]

        seen[curr_state] = cost

        for succ in problem.getSuccessors(curr_state):
            state = succ[0]
            actions = curr[1][:]
            # update the new cost for the next step
            new_cost = cost + succ[2]

            # if the state is not already explored (path checking)
            if state not in seen or new_cost < seen[state]:

                actions.append(succ[1])
                item = [state, actions, new_cost]
                # print("item: ", item)
                front.push(item, new_cost)
                seen[state] = new_cost

    # if there is no solution
    return False


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.

    test
    python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
    """
    front = util.PriorityQueue()

    # initial state has no action (the steps needed to the current state)
    front.push([problem.getStartState(), [], 0], 0)

    # keep track all the states that has been explored
    seen = {}

    while not front.isEmpty():
        curr = front.pop()
        curr_state = curr[0]
        cost = curr[2]
        # print(curr)

        if problem.isGoalState(curr_state):
            return curr[1]

        seen[curr_state] = cost

        for succ in problem.getSuccessors(curr_state):
            state = succ[0]
            actions = curr[1][:]
            # update the new cost for the next step
            new_cost = cost + succ[2]

            # if the state is not already explored (path checking)
            if state not in seen or new_cost < seen[state]:
                actions.append(succ[1])
                item = [state, actions, new_cost]
                # print("item: ", item)
                front.push(item, heuristic(state, problem) + new_cost)
                # print(heuristic(state, problem))
                seen[state] = new_cost

    # if there is no solution
    return False


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
