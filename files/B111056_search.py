# -*- coding: cp949 -*-
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


def iterativeDeepeningSearch(problem):
    """
    depth�� 1�� ������Ű�鼭 ���� ��θ� ã�� �� ���� �ݺ�
    :param problem:
    :return: start state�κ��� goal������ action��
    """
    depth = 0
    while True:
        action = []
        if travelRecursively(problem, problem.getStartState(), depth, [problem.getStartState()], action):
            return action
        depth += 1


def travelRecursively(problem, state, depth, visited, actions):
    """
    state�� ���� goal���� ���� ���� ���θ� ��ȯ�ϰ� ���ް����ϴٸ� start state�� ���� goal������ action���� actions�� �ݿ��ȴ�.

    :param problem: problem
    :param state: current state
    :param depth: current limit depth
    :param visited: visited states
    :param actions: actions
    :return: current state�� ���� goal���� ���� ���� ����(Boolean), �湮�� state�� �ٽ� �湮���� �ʴ´�.
    """
    if problem.isGoalState(state):
        return True

    if depth <= 0:
        return False

    for child in problem.getSuccessors(state):
        if child[0] in visited:
            continue

        visited.append(child[0])
        actions.append(child[1])
        result = travelRecursively(problem, child[0], depth - 1, visited, actions)
        if result:
            print(visited)
            return True
        visited.pop()
        actions.pop()

    return False


def nullHeuristic(state, problem=None):
    return 0


# goal state�� 1,1 �̶�� �����ϰ� heuristic�� ���� state���� goal������ manhattan distance ����
def customHeuristic(state, problem=None):
    if problem.isGoalState(state):
        return 0
    return state[0] + state[1] - 2


def aStarSearch(problem, heuristic=customHeuristic):
    """
    A* algorithm���� start state�κ��� goal������ actions���� ����ϰ� ��ȯ
    Expanded Tree�� f(n)�� �ּҰ� �ǰ� goal state�� leaf node�� ã�������� tree�� ��� expand�Ѵ�
         
    :param problem: problem
    :param heuristic: ����� heuristic function
    :return: start state�κ��� goal������ action��
    """

    class Leaf:
        """
        Leaf of expanded tree

        Members:
            state: current State
            prevState: previous State
        """

        def __init__(self, parentLeaf, state, action, cost):
            self.state = state
            self.prevState = None if parentLeaf is None else parentLeaf.state

            self._totalCoast = cost if parentLeaf is None else (parentLeaf.getGn() + cost)  # set g(n)
            # <editor-fold desc="parent ������ actions�� �������� ����� ���� action�� ����, memory save">
            self._prevAction = action
            if parentLeaf is None:
                self._actionsUntilParent = []
            else:
                self._actionsUntilParent = parentLeaf.getTotalActions()
            # </editor-fold>

        def getGn(self):
            """
            :return: g(n): cost so far to reach n
            """
            return self._totalCoast

        def getFn(self):
            """
            :return: f(n):estimated total cost of path through n to goal.
            """
            return heuristic(self.state, problem) + self._totalCoast

        def getTotalActions(self):
            """
            :return start state���� current state������ actions:
            """
            newActions = list(self._actionsUntilParent)
            if self._prevAction is not None:
                newActions.append(self._prevAction)
            return newActions

    leafs = [Leaf(None, problem.getStartState(), None, 0)]  # �ʱ� Leafs ����

    def expand():
        """
        f(n) ���� ���� ���� node�� Ȯ���Ų��.
        :return: Ȯ�尡�� ���θ� ��ȯ. False ��ȯ�� goal state��
        """
        global goalActions
        removeLeaf = None

        # <editor-fold desc="������ leaf Ž�� f(n)�� �ּ��� node ã��">
        for leaf in leafs:
            if removeLeaf is None:
                removeLeaf = leaf
                continue
            if removeLeaf.getFn() > leaf.getFn():
                removeLeaf = leaf
        # </editor-fold>

        if removeLeaf is None:
            return False

        if problem.isGoalState(removeLeaf.state):
            goalActions = removeLeaf.getTotalActions()
            return False

        leafs.remove(removeLeaf)
        for (state, action, stepCost) in problem.getSuccessors(removeLeaf.state):
            if state == removeLeaf.prevState:
                continue
            newLeaf = Leaf(removeLeaf, state, action, stepCost)
            leafs.append(newLeaf)

        return True

    while expand():
        # print(map(lambda x: x.getFn(), leafs))
        pass
    return goalActions


# Abbreviations
ids = iterativeDeepeningSearch
astar = aStarSearch
