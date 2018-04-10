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
    depth를 1씩 증가시키면서 도착 경로를 찾을 때 까지 반복
    :param problem:
    :return: start state로부터 goal까지의 action들
    """
    depth = 0
    while True:
        action = []
        if travelRecursively(problem, problem.getStartState(), depth, [problem.getStartState()], action):
            return action
        depth += 1


def travelRecursively(problem, state, depth, visited, actions):
    """
    state로 부터 goal까지 도달 가능 여부를 반환하고 도달가능하다면 start state로 부터 goal까지의 action들이 actions에 반영된다.

    :param problem: problem
    :param state: current state
    :param depth: current limit depth
    :param visited: visited states
    :param actions: actions
    :return: current state로 부터 goal까지 도달 가능 여부(Boolean), 방문한 state는 다시 방문하지 않는다.
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


# goal state가 1,1 이라고 가정하고 heuristic을 현재 state에서 goal까지의 manhattan distance 설정
def customHeuristic(state, problem=None):
    if problem.isGoalState(state):
        return 0
    return state[0] + state[1] - 2


def aStarSearch(problem, heuristic=customHeuristic):
    """
    A* algorithm으로 start state로부터 goal까지의 actions들을 계산하고 반환
    Expanded Tree의 f(n)이 최소가 되고 goal state인 leaf node를 찾을때까지 tree를 계속 expand한다
         
    :param problem: problem
    :param heuristic: 사용할 heuristic function
    :return: start state로부터 goal까지의 action들
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
            # <editor-fold desc="parent 까지의 actions와 직전에서 현재로 가는 action을 저장, memory save">
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
            :return start state부터 current state까지의 actions:
            """
            newActions = list(self._actionsUntilParent)
            if self._prevAction is not None:
                newActions.append(self._prevAction)
            return newActions

    leafs = [Leaf(None, problem.getStartState(), None, 0)]  # 초기 Leafs 설정

    def expand():
        """
        f(n) 값이 제일 작은 node를 확장시킨다.
        :return: 확장가능 여부를 반환. False 반환시 goal state가
        """
        global goalActions
        removeLeaf = None

        # <editor-fold desc="삭제할 leaf 탐색 f(n)이 최소인 node 찾기">
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
