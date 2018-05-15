
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import numpy as np
from operator import itemgetter
from cozmo.util import degrees, distance_mm, speed_mmps
from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id
import time



def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start = grid.getStart()
    goals = grid.getGoals()

    row = grid.width
    col = grid.height
    
    #return if start point is out of bounds

    if not grid.coordInBounds(start):
        pass
    
    #return if any of the goals is out of bounds
    goalInBound = True
    for goal in goals:
        goalInBound &= grid.coordInBounds(goal)

    if not goalInBound:
        pass

    #return if start is blocked

    if start in grid._obstacles:
        pass

    #return if goals is blocked
    goalBlocked = False
        
    for goal in goals:
        goalBlocked |= goal in grid._obstacles

    if goalBlocked:
        pass

    #if start == goal return

    if start in goals:
        pass

    #defaultCell = CellInfo(math.inf, math.inf, math.inf, (-1,-1))
    defaultCell = CellInfo()
    defaultCell.updateCost(math.inf, math.inf, math.inf)
    defaultCell.updateParent((-1,-1))
    #print("defaultCell" + str(defaultCell))

    cellInfo = []
    #cellInfo = [[defaultCell for x in range(row)] for y in range(col)]

    for x in range(row):
        cols = []
        for y in range(col):
            newCell = CellInfo()
            newCell.updateCost(math.inf, math.inf, math.inf)
            newCell.updateParent((-1,-1))
            cols.append(newCell)
        cellInfo.append(cols)

    #print("defaultCell" + str(cellInfo[0][0]))
    cellInfo[start[0]][start[1]].updateCost(0,0,0)
    cellInfo[start[0]][start[1]].updateParent((start))
    #print("defaultCell" + str(cellInfo[0][0]))
    unvisited =[]
    unvisited.append((start[0], start[1], cellInfo[start[0]][start[1]]))

    #print("Neighbours " + str(grid.getNeighbors(start)))
    #s = sorted(grid.getNeighbors(start), key=itemgetter(1), reverse=True)
    #print("Sorted Neighbours " + str(s))

    foundGoal = False
    while unvisited:
        current = unvisited.pop()
        grid.addVisited((current[0], current[1]))
        #print("current" + str((current[0], current[1])))
        neighbors = grid.getNeighbors((current[0], current[1]))
        n_w = []
        for neighbour in neighbors:#sorted(neighbors, key=itemgetter(1)):
            if isGoal(neighbour[0], goals):
                print("found goal")
                n_x, n_y = neighbour[0]
                cellInfo[n_x][n_y].updateParent((current[0],current[1]))
                foundGoal = True
                break
            if neighbour[0] not in grid.getVisited():
                #print("first valid neighbour" + str((neighbour[0], neighbour[1])))
                newG = current[2].fgh[1] + neighbour[1]
                newH = heuristic(neighbour[0], goals[0])
                newF = newG + newH
                n_x, n_y = neighbour[0]
                n_cell = cellInfo[n_x][n_y]
                #print("oldf " + str(n_cell.fgh[0]) + "newF " + str(newF))
                #print("newf " + str(newF))
                if n_cell.fgh[0] > newF:
                    #print("adding to unvisitedlist" + str((n_x,n_y)))
                    cellInfo[n_x][n_y].updateCost(newF, newG, newH)
                    cellInfo[n_x][n_y].updateParent((current[0],current[1]))
                    n_w.append(((n_x, n_y), newF))
        n_w_s = sorted(n_w, key=itemgetter(1), reverse=True)
        for ((new_x, new_y), new_f) in n_w_s:
            unvisited.append((new_x, new_y, cellInfo[new_x][new_y]))
        if foundGoal:
            break

    t_x = goals[0][0]
    t_y = goals[0][1]

    mypath = []
    next_cell = cellInfo[t_x][t_y]
    #print("Goal  " + str(goals[0]))
    #print("Goal Parent " + str(next_cell.parent))
    #print("Start " + str(start))
    #print("Start parent " + str(cellInfo[start[0]][start[1]].parent))
    #while next_cell.parent[0] != start[0] & next_cell.parent[1] != start[1]:
    mypath.append(goals[0])
    while next_cell.parent != start:
        #print("Parent1 " + str(next_cell.parent))
        mypath.append(next_cell.parent)
        next_cell = cellInfo[next_cell.parent[0]][next_cell.parent[1]]

    mypath.append(start)
    mypath.reverse()
    #print(mypath)
    path = []
    for visited in grid.getVisited():
        path.append((visited[0],visited[1]))

    #print(path)
    grid.setPath(mypath)
    #print(grid.getPath())

    pass # Your code here

def isGoal(current, goals):
    if current in goals:
        return True
    return False

def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    #dist = math.fabs(math.sqrt((math.pow((current[0] - goal[0]),2) + math.pow((current[1] - goal[1]),2))))
    dist = max(math.fabs(current[0] - goal[0]), math.fabs(current[1] - goal[1]))
    return dist # Your code here

def euclideanDistance(current, goal):
    dist = math.fabs(math.sqrt((math.pow((current[0] - goal[0]),2) + math.pow((current[1] - goal[1]),2))))
    return dist

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    robot.move_lift(-1)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    robot.world.connect_to_cubes()
    robot.world.auto_disconnect_from_cubes_at_end()
    time.sleep(5) 

    
    bh = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    robot.world.wait_for_observed_light_cube()
    bh.stop()
    robot.turn_in_place(degrees(-robot.pose.rotation.angle_z.degrees)).wait_for_completed()

    start = grid.getStart()
    scale = grid.scale

    currentPosition = start
    currentAngle = 0
    goal_obs_time = None
    obstacle1_obs_time = None
    obstacle2_obs_time = None

    obstacles = {}
    #print("Starting caliberation")
    while not stopevent.is_set():
        goal = robot.world.get_light_cube(LightCube1Id) 

        #print("GoalObserved" + str(goal_obs_time))
        if goal_obs_time is None: 
            goal_obs_time = goal.last_observed_time
            grid.clearGoals()
            grid.clearObstacles()
            goal_grid = objectToGridCoords(goal, scale, start)
            obstacles['g'] = padCubes(goal_grid, grid)
            grid.addObstacles([itemm for sublist in obstacles.values() for item in sublist])
            goal_grid = (goal_grid[0], goal_grid[1]) 
            #print("Goal ", str(goal_grid))
            grid.addGoal(goal_grid)

        obstacle1 = robot.world.get_light_cube(LightCube2Id)
        if (obstacle1 and obstacle1_obs_time is None):
            grid.clearObstacles()
            obstacle1_obs_time = obstacle1.last_observed_time
            obstacles['obs1'] = padCubes(objectToGridCoords(obstacle1, scale, start), grid)
            grid.addObstacles([item for sublist in obstacles.values() for item in sublist])

        obstacle2 = robot.world.get_light_cube(LightCube3Id)
        if (obstacle2 and obstacle2_obs_time is None):
            obstacle2_obs_time = obstacle2.last_observed_time
            obstacles['cube2'] = padCubes(objectToGridCoords(obstacle2, scale, start), grid)
            grid.clearObstacles()
            grid.addObstacles([item for sublist in obstacles.values() for item in sublist])

        grid.clearPath()
        grid.setStart(currentPosition)
        if len(grid.getGoals()) > 0:
            astar(grid, heuristic)

        if (len(grid.getPath())>2):
            toCell = grid.getPath()[1]
            currentAngle = navigate(robot, currentPosition, currentAngle, toCell, grid)
            currentPosition = toCell

def navigate(robot, curCell, curOrientation, toCell, grid):
    offset = (toCell[0]-curCell[0], toCell[1]-curCell[1])
    angle = angleFromOffset(offset)
    turn = angle-curOrientation
    if turn>180:
        turn = turn-360
    elif turn<-180:
        turn = turn+360

    robot.turn_in_place(degrees(turn)).wait_for_completed()
    mm = euclideanDistance((0,0), offset) * grid.scale
    robot.drive_straight(distance_mm(mm), speed_mmps(mm)).wait_for_completed()
    return angle

def angleFromOffset(offset):
    return {
        (1,0): 0,
        (1,1): 45,
        (0,1): 90,
        (-1,1): 135,
        (-1,0): 180,
        (-1,-1): 225,
        (0,-1): 270,
        (1,-1): 315
    }[offset]


def padCubes(objCoords, grid):
    obstacles = []
    for i in range(-2, 3, 1):
        for j in range(-2, 3, 1):
            cell = (objCoords[0] + i, objCoords[1] + j)
            if grid.coordInBounds(cell) and cell not in obstacles:
                obstacles.append(cell)
    return obstacles

def objectToGridCoords(lightCube, scale, origin):
    x = lightCube.pose.position.x // scale + origin[0]
    y = lightCube.pose.position.y // scale + origin[1]
    return (math.floor(x), math.floor(y))


class CellInfo(object):
    #f = "Full cost from start to goal via this cell"
    #g = "Cost from start to this cell"
    #h = "Cost from this cell to goal"
    #parent_x = "parent x cell of this cell"
    #parent_y = "parent y cell of this cell"

    def __init__(self, f=None, g=None, h=None, parent=None):
        if parent is not None:
            self.parent_x = parent[0]
            self.parent_y = parent[1]
        self.f = f
        self.g = g
        self.h = h

    def __repr__(self):
        return "(f = %f, g = %f, h = %f )" % (self.f, self.g, self.h)

    @property
    def parent(self):
        return (self.parent_x, self.parent_y)

    @property
    def fgh(self):
        return self.f, self.g, self.h

    def updateCost(self, f=None, g=None, h=None):
        if f is not None:
            self.f = f
        if g is not None:
            self.g = g
        if h is not None:
            self.h = h

    def updateParent(self, parent=None):
        if parent is not None:
            self.parent_x = parent[0]
            self.parent_y = parent[1]

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

