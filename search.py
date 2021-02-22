import sys
import timeit

import util


class SearchProblem:


    def getStartState(self):

        util.raiseNotDefined()

    def isGoalState(self, state):

        util.raiseNotDefined()

    def getSuccessors(self, state):

        util.raiseNotDefined()

    def getCostOfActions(self, actions):

        util.raiseNotDefined()


def tinyMazeSearch(problem):

    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def derinlikOncelikliArama(problem):
    start_time = timeit.default_timer()
    print "Start_time",start_time
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    stack1 = util.Stack()
    gezilen = []
    node = {}


    node["ebeveyn"] = None
    node["hareket"] = None
    node["durum"] = problem.getStartState()

    stack1.push(node)

    while (True):
        if (stack1.isEmpty()):
            sys.exit('failure')

        node = stack1.top()
        durum = node["durum"]

        if (durum in gezilen):
            stack1.pop()
            continue

        gezilen.append(durum)

        if (problem.isGoalState(durum)):
            break

        cocuk = problem.getSuccessors(durum)

        if (cocuk):
            stack1.pop()
            for i in range(len(cocuk)):
                if (cocuk[i][0] not in gezilen):
                    altNode = {}
                    altNode["ebeveyn"] = node
                    altNode["hareket"] = cocuk[i][1]
                    altNode["durum"] = cocuk[i][0]
                    stack1.push(altNode)
        else:
            stack1.pop()

    yol = []
    while (node["hareket"] != None):
        yol.insert(0, node["hareket"])
        node = node["ebeveyn"]
    elapsed = timeit.default_timer() - start_time
    print("time", elapsed)
    return yol

    util.raiseNotDefined()


def genislikOncelikliArama(problem):


    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    queue1 = util.Queue()
    gezilen = []
    node = {}



    node["ebeveyn"] = None
    node["hareket"] = None
    node["durum"] = problem.getStartState()

    queue1.push(node)

    while (True):
        if (queue1.isEmpty()):
            sys.exit('failure')

        node = queue1.top()
        durum = node["durum"]

        if (problem.isGoalState(durum)):
            break

        if (durum in gezilen):
            queue1.pop()
            continue

        gezilen.append(durum)

        cocuk = problem.getSuccessors(durum)

        if (cocuk):
            queue1.pop()  # enqueue
            for i in range(len(cocuk)):

                if (cocuk[i][0] not in gezilen):
                    altNode = {}
                    altNode["ebeveyn"] = node
                    altNode["hareket"] = cocuk[i][1]
                    altNode["durum"] = cocuk[i][0]
                    queue1.push(altNode)
        else:
            queue1.pop()

    yol = []
    while (node["hareket"] != None):
        yol.insert(0, node["hareket"])
        node = node["ebeveyn"]

    return yol

    util.raiseNotDefined()


def dusukMaliyetliArama(problem):

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    priorityQueue1 = util.PriorityQueue()
    ziyaret = []
    node = {}



    node["ebeveyn"] = None
    node["hareket"] = None
    node["maliyet"] = 0
    node["durum"] = problem.getStartState()

    priorityQueue1.push(node, node["maliyet"])

    while (True):

        if (priorityQueue1.isEmpty()):
            sys.exit('failure')

        node = priorityQueue1.pop()
        durum = node["durum"]

        if (problem.isGoalState(durum)):
            break

        if (durum in ziyaret):
            continue

        ziyaret.append(durum)
        cocuk = problem.getSuccessors(durum)

        if (cocuk):
            for i in range(len(cocuk)):

                if (cocuk[i][0] not in ziyaret):
                    altNode = {}
                    altNode["ebeveyn"] = node
                    altNode["hareket"] = cocuk[i][1]
                    altNode["durum"] = cocuk[i][0]
                    altNode["maliyet"] = cocuk[i][2] + node["maliyet"]

                    priorityQueue1.push(altNode, altNode["maliyet"])

    yol = []
    while (node["hareket"] != None):
        yol.insert(0, node["hareket"])
        node = node["ebeveyn"]

    return yol

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):

    return 0


def aStarArama(problem, heuristic=nullHeuristic):

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    priorityQueue2 = util.PriorityQueue()
    gezilenA = []
    node = {}



    node["ebeveyn"] = None
    node["hareket"] = None
    node["hedef"] = 0
    node["heuristic"] = heuristic(problem.getStartState(), problem)
    node["durum"] = problem.getStartState()

    priorityQueue2.push(node, node["hedef"] + node["heuristic"])

    while (True):

        if (priorityQueue2.isEmpty()):
            sys.exit('failure')

        node = priorityQueue2.pop()
        durum = node["durum"]

        if (problem.isGoalState(durum)):
            break

        if (durum in gezilenA):
            continue

        gezilenA.append(durum)
        cocuk = problem.getSuccessors(durum)

        if (cocuk):
            for i in range(len(cocuk)):

                if (cocuk[i][0] not in gezilenA):
                    altNode = {}
                    altNode["ebeveyn"] = node
                    altNode["hareket"] = cocuk[i][1]
                    altNode["durum"] = cocuk[i][0]
                    altNode["hedef"] = cocuk[i][2] + node["hedef"]
                    altNode["heuristic"] = heuristic(altNode["durum"], problem)
                    priorityQueue2.push(altNode, altNode["hedef"] + altNode["heuristic"])

    yol = []
    while (node["hareket"] != None):
        yol.insert(0, node["hareket"])
        node = node["ebeveyn"]

    return yol

    util.raiseNotDefined()



goa = genislikOncelikliArama
doa = derinlikOncelikliArama
astar = aStarArama
dma = dusukMaliyetliArama
