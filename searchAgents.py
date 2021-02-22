# coding=utf-8
import timeit

from game import Directions
from game import Agent
from game import Actions
import util
import time
import search
from search import aStarArama


class GoWestAgent(Agent):
    "Kodun bu kismi Dan Klein ve John DeNero tarafindan yazilmistir."

    def getAction(self, state):

        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP



class SearchAgent(Agent):
    """
    Genel kullanima yönelik bir ajandir. Arama yapilmak istenen problem icin gerekli algoritma ile baglanti kurar.
    Default olarak derinlik oncelikli arama icin calisacaktir. Default arama disinda bir metot kullanmak icin
    fn ile belirtilmelidir.
    Not: SearchAgent icinde degisiklik yapilmamalidir. Bu kisim John DeNero ve Dan Klein tarafindan yazilmistir.
    """

    def __init__(self, fn='derinlikOncelikliArama', prob='PositionSearchProblem', heuristic='nullHeuristic'):

        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))

            self.searchFunction = lambda x: func(x, heuristic=heur)


        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        Ajanin layoutu ilk kez gordugu kisimdir. Hedefe giden yol cizilir. Gerekli hesaplamalar yapilir.
        """
        start_time = timeit.default_timer()
        print "Start_time", start_time
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)



    def getAction(self, state):
        """
        Secilen yoldaki sonraki hareketi alacak sekilde tasarlanmistir. Yapilabilecek hareket kalmamissa durdurur.
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class PositionSearchProblem(search.SearchProblem):
    """
    Bir arama problemi durum uzayini, baslangic durumunu, hedef testini, successor islevi ve maliyet islevini tanimlar.
    Bu arama problemi, pacman panosundaki belirli bir noktaya giden yolları bulmak icin kullanilabilir.
    Durum uzayi, bir pacman oyununda (x, y) konumlarindan olusur.

    Note:PositionSearchProblem tam anlamiyla belirtilmistir ve degistirilmemelidir. Bu kisim John DeNero ve Dan Klein tarafindan yazilmistir.
    """

    def __init__(self, gameState, costFn=lambda x: 1, goal=(1, 1), start=None, warn=True, visualize=True):

        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print 'Warning: this does not look like a regular search maze'

        self._visited, self._visitedlist, self._expanded = {}, [], 0

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal


        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display):
                    __main__._display.drawExpandedCells(self._visitedlist)

        return isGoal

    def getSuccessors(self, state):

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append((nextState, action, cost))


        self._expanded += 1
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):

        if actions == None: return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x, y))
        return cost


class StayEastSearchAgent(SearchAgent):
    """
    Tahtanin bati tarafinda bulunma durumunda, maliyet duzenleyen ajan
    Maliyet 1/2^n

    """

    def __init__(self):
        self.searchFunction = search.dusukMaliyetliArama
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)


class StayWestSearchAgent(SearchAgent):
    """
    Tahtanin bati tarafinda bulunma durumunda, maliyet duzenleyen ajan
    Maliyet 2^n
    """

    def __init__(self):
        self.searchFunction = search.dusukMaliyetliArama
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)


def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance"
    xy1 = position
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5




class CornersProblem(search.SearchProblem):
    """
    Dort kose icin bulunan layout uzerinde gezinme islemi icin tasarlandi

    """

    def __init__(self, startingGameState):

        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height - 2, self.walls.width - 2
        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print 'Warning: no food in corner ' + str(corner)
        self._expanded = 0  # DO NOT CHANGE; Number of search nodes expanded
       #Problem icin baslangıic durumu
        self.corners_flags = [0, 0, 0, 0]
        self.start = (self.startingPosition, self.corners_flags)

    def getStartState(self):

        return self.start
        util.raiseNotDefined()

    def isGoalState(self, state):

        isGoal = not (0 in state[1])
        return isGoal
        util.raiseNotDefined()

    def getSuccessors(self, state):

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            hitsWall = self.walls[nextx][nexty]

            if not hitsWall:

                durum = (nextx, nexty)

                
                gezilen_corner = list(state[1])
                if durum == self.corners[0]:
                     gezilen_corner[0] = True

                if durum == self.corners[1]:
                    gezilen_corner[1] = True

                if durum == self.corners[2]:
                    gezilen_corner[2] = True

                if durum == self.corners[3]:
                    gezilen_corner[3] = True

                maliyet = 1
                successors.append(((durum, tuple(gezilen_corner)), action, maliyet,))
        self._expanded += 1
        return successors

    def getCostOfActions(self, actions):
        """
        Hareket dizisinin maliyetini dondurur. Gecerli olmayan hareket yapilirsa, 999999 dondurur.
        Kodun bu kismi Dan Klein ve John DeNero tarafindan yazilmistir.
        """
        if actions == None: return 999999
        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)




def cornersHeuristic(state, problem):
    corners = problem.corners
    walls = problem.walls

    h = 0

    ziyaret_kose = []
    dot = state[0]
   


    for k in corners:
        if k in state[1]:
            break
        else:

            ziyaret_kose.append(k)


    while ziyaret_kose:
        mesafe, k = min([(util.manhattanDistance(dot, k), k) \
                         for k in ziyaret_kose])
        h += mesafe
        dot = k
        ziyaret_kose.remove(k)

    return h


class AStarCornersAgent(SearchAgent):

    def __init__(self):
        self.searchFunction = lambda prob: search.aStarArama(prob, cornersHeuristic)
        self.searchType = CornersProblem


class FoodSearchProblem:

    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0
        self.heuristicInfo = {}

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):

        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):

        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class AStarFoodSearchAgent(SearchAgent):


    def __init__(self):
        self.searchFunction = lambda prob: search.aStarArama(prob, foodHeuristic)
        self.searchType = FoodSearchProblem


def foodHeuristic(state, problem):
    currentPosition, foodGrid = state

    h=0
    for food in foodGrid.asList():
        h=(mazeDistance(currentPosition,food,problem.startingGameState))

    return h


class ClosestDotSearchAgent(SearchAgent):

    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while (currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState)  # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print 'Path found with cost %d.' % len(self.actions)

    def findPathToClosestDot(self, gameState):
        problem = AnyFoodSearchProblem(gameState)
        path = aStarArama(problem)
        return path
        util.raiseNotDefined()


class AnyFoodSearchProblem(PositionSearchProblem):


    def __init__(self, gameState):

        self.food = gameState.getFood()
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def isGoalState(self, state):

        x, y = state
        if (self.food[x][y]):
            return True
        return False
        util.raiseNotDefined()


def mazeDistance(point1, point2, gameState):

    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.goa(prob))
