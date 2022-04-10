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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """ Search the deepest nodes in the search tree first. """
    "*** ALGORITMO GRAPH-SEARCH DEL CAP. 3 ***"
    # =============== ESTRUCTURA DE DATOS A USAR =========================
    # pila que contendra el nodo/dict (state, actions) del agente
    # usamos una pila ya que es necesario evaluar a profundidad, es decir, extender
    # de los ultimos nodos succesores o adyacentes insertados. (LIFO)
    fringe = util.Stack()
    # una lista de los estados visitados para evitar bucles
    closed = []
    # inicializamos la pila con el estado inicial
    fringe.push((problem.getStartState(), []))
    # =============== BUSQUEDA =========================
    while not fringe.isEmpty():
        state, actions = fringe.pop()
        # verficamos si estamos en el estado fin
        if problem.isGoalState(state):
            return actions
        # verificamos para evitar bucles antes de expandir
        if state not in closed:
            closed.append(state)
            # obtenemos todos los nodos sucesores o adyacentes del nodo actual
            for successor in problem.getSuccessors(state):
                # successor[0] -> sgte. estado, successor[1] -> accion, actions + [successor[1]] -> sgte. accion
                fringe.push((successor[0], actions + [successor[1]]))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** ALGORITMO GRAPH-SEARCH DEL CAP. 3 ***"
    # =============== ESTRUCTURA DE DATOS A USAR =========================
    # cola que contendra el conjunto/dict (state/node, actions) del agente
    # usamos una cola ya que es necesario evaluar a los primeros nodos sucesores
    # del nodo que acabamos de visitar. (FIFO)
    fringe = util.Queue()
    # una lista de los nodos visitados para evitar bucles
    closed = []
    # inicializamos la cola con el estado inicial
    fringe.push((problem.getStartState(), []))
     # =============== BUSQUEDA =========================
    while not fringe.isEmpty():
        state, actions = fringe.pop()
        # verifcamos si estamos en el estado fin
        if problem.isGoalState(state):
            return actions
        # verificamos para evitar bucles antes de expandir
        if state not in closed:
            closed.append(state)
            # obtenemos todos los nodos sucesores o adyacentes del nodo actual
            for successor in problem.getSuccessors(state):
                # successor[0] -> sgte. estado, successor[1] -> accion, actions + [successor[1]] -> sgte. accion
                fringe.push((successor[0], actions + [successor[1]]))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** ALGORITMO GRAPH-SEARCH DEL CAP. 3 ***"
    # =============== ESTRUCTURA DE DATOS A USAR =========================
    # cola que contendra el nodo/dict (state, actions, cost) del agente
    # usamos esta priorityQueue porque es necesario que a cada sgte. nodo con la
    # sgte. accion tenga una prioridad para saber que nodo sucesor sera mas optimo extender.
    fringe = util.PriorityQueue()
    # una lista de los estados visitados para evitar bucles
    closed = []
    # inicializamos la cola con el estado inicial con la prioredad menor posible
    fringe.push((problem.getStartState(), [], 0), 0)
     # =============== BUSQUEDA =========================
    while not fringe.isEmpty():
        state, actions, currentCost = fringe.pop()
        # verifcamos si estamos en el estado fin
        if problem.isGoalState(state):
            return actions
        # verificamos para evitar bucles antes de expandir
        if state not in closed:
            closed.append(state)
            # expandimos los nodos con el costo de "ruta" mas bajo (prioridad mas baja)
            for successor in problem.getSuccessors(state):
                nextState, action, cost = successor
                priority = currentCost + cost
                fringe.push((nextState, actions + [action], priority), priority)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "*** ALGORITMO GRAPH-SEARCH DEL CAP. 3 ***"
    # =============== ESTRUCTURA DE DATOS A USAR =========================
    # cola que contendra el nodo ((state, actions, cost), prioredad)
    # usamos esta priorityQueue porque es necesario manejar la prioredad de la cola
    # con el costo de la heuristica, asi expandiremos el nodo mas optimo de acuerdo al
    # costo de la heuristica.
    fringe = util.PriorityQueue()
    # una lista de los estados visitados para evitar bucles
    closed = []
    # inicializamos la cola con el estado inicial con la prioredad menor posible
    fringe.push((problem.getStartState(), [], 0), 0)

    while not fringe.isEmpty():
        state, actions, currentCost = fringe.pop()
        # verifcamos si estamos en el estado fin
        if problem.isGoalState(state):
            return actions
        # verificamos para evitar bucles antes de expandir
        if state not in closed:
            closed.append(state)
            # expandimos el nodo con costo de heuristica mas optima
            for successor in problem.getSuccessors(state):
                nextState, action, cost = successor
                newCost = currentCost + cost
                # f(n) = g(n) + h(n) = cost to n + heuristic
                heuristicCost = newCost + heuristic(nextState, problem)
                # insertamos el siguiente nodo con su costo heuristico (la prioredad)
                fringe.push((nextState, actions + [action], newCost), heuristicCost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
