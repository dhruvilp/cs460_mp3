"""
NAMES: DHRUVIL (DHP68) & SEAN (SD997)
"""

import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''

def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    ax.set_axis_off()
    return fig, ax


'''
Make a patch for a single pology
'''

def createPolygonPatch(polygon, color):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch


'''
Render the problem
'''

def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()


'''**********************************************************'''

'''
Grow a simple RRT
'''

def calculateDistance(point1, point2):
    x1 = point1[0]
    y1 = point1[1]
    x2 = point2[0]
    y2 = point2[1]
    distance = np.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    return distance


def nearestPoint(newPoints, x_new):
    x_near = -1
    distanceVal = 15
    for i, coordinates in newPoints.items():
        if newPoints[i] != newPoints[x_new]:
            distance_new = calculateDistance(newPoints[x_new], newPoints[i])
            if distance_new < distanceVal:
                distanceVal = distance_new
                x_near = i

    return x_near, distanceVal


def distance(p1, p2):
    return np.sqrt(np.sum(np.power(np.subtract(p1, p2), 2)))


def nst(p, newPoints):
    if len(newPoints) == 0:
        return None
    else:
        n = 1
        for x in newPoints:
            if distance(p, newPoints[x]) < distance(p, newPoints[n]):
                n = x
    return n


def perpendicular_function(p_p, p_o):
    if p_p[1] - p_o[1] != 0:
        a = -(p_p[0] - p_o[0]) / (p_p[1] - p_o[1])
        b = p_p[1] - a * p_p[0]
        return a, b
    else:
        return None, None


def normal_function(p_p, p_o):
    if p_p[0] - p_o[0] != 0:
        a = (p_o[1] - p_p[1]) / (p_o[0] - p_p[0])
        b = p_o[1] - a * p_o[0]
        return a, b
    else:
        return None, None


def getprojection(a, b, c):
    pa_a, pa_b = perpendicular_function(a, b)
    pb_a, pb_b = perpendicular_function(b, a)
    n_a, n_b = normal_function(a, b)
    if pa_a is None:
        if b[0] <= c[0] <= a[0] or a[0] <= c[0] <= b[0]:
            out = (c[0], a[1])
            return [distance(out, c), out, 0]
    elif pa_a == 0:
        if b[1] <= c[1] <= a[1] or a[1] <= c[0] <= b[0]:
            out = (a[0], c[1])
            return [distance(out, c), out, 0]
    else:
        if (c[1] - (pa_a * c[0] + pa_b)) * (c[1] - (pb_a * c[0] + pb_b)) < 0:
            t_b = c[1] - pa_a * c[0]
            x = (n_b - t_b) / (pa_a - n_a)
            y = n_a * x + n_b
            return [distance((x, y), c), (x, y), 0]
    if distance(a, c) > distance(b, c):
        return [distance(b, c), None, 2]
    else:
        return [distance(a, c), None, 1]


def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()
    extra = len(points) + 1
    for x in points:
        if x not in newPoints:
            if x == 1:
                adjListMap[x] = []
                newPoints[x] = points[x]
            elif x == 2:
                adjListMap[1].append(2)
                newPoints[2] = points[x]
                adjListMap[2] = [1]
            else:
                best = [15, None, []]
                for nearest_p in adjListMap:
                    for pre_p in adjListMap[nearest_p]:
                        templist = getprojection(newPoints[nearest_p], newPoints[pre_p], points[x])
                        if templist[0] < best[0]:
                            best[0] = templist[0]
                            if templist[2] == 0:
                                best[1] = templist[1]
                                best[2] = [nearest_p, pre_p]
                            elif templist[2] == 1:
                                best[1] = None
                                best[2] = [nearest_p]
                            else:
                                best[1] = None
                                best[2] = [pre_p]
                if best[1] is None:
                    newPoints[x] = points[x]
                    adjListMap[x] = [best[2][0]]
                    adjListMap[best[2][0]].append(x)
                else:
                    a = best[2][1]
                    b = best[2][0]
                    newPoints[x] = points[x]
                    newPoints[extra] = best[1]
                    adjListMap[x] = [extra]
                    adjListMap[a].remove(b)
                    adjListMap[b].remove(a)
                    adjListMap[a].append(extra)
                    adjListMap[b].append(extra)
                    adjListMap[extra] = [a, b, x]
                    extra = extra + 1
    return newPoints, adjListMap


'''
Perform basic search
'''


def basicSearch(tree, start, goal):
    path = []
    # YOUR CODE GOES HERE
    visited = dict()
    for key, pairs in tree.items():
        visited[key] = False

    queue = [start]
    backpointers = dict()
    while len(queue) != 0:
        key = queue.pop(0)
        visited[key] = True
        if key != goal:
            for adjNode in tree[key]:
                if not visited[adjNode]:
                    queue = [adjNode] + queue
                    backpointers[adjNode] = key
        else:
            break

    path.append(goal)
    while path[0] != start:
        path = [backpointers[path[0]]] + path

    if path[len(path) - 1] != goal:
        print("No path found!")

    return path

'''
Display the RRT and Path
'''


def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
    # YOUR CODE GOES HERE
    if robotStart and robotGoal and polygons:
        _, ax = setupPlot()
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    path_edges = set([(path[i], path[i + 1]) for i in range(len(path) - 1)])
    visited = set()

    for (v1, adj) in adjListMap.items():
        for v2 in adj:
            if (v1, v2) not in visited and (v2, v1) not in visited:
                visited.add((v1, v2))
                if (v1, v2) in path_edges or (v2, v1) in path_edges:
                    plt.plot(*zip(points[v1], points[v2]), color="orange")
                else:
                    plt.plot(*zip(points[v1], points[v2]), color="black")
            # print('*********')

    plt.show()


'''
Collision checking
'''


def isCollisionFree(robot, point, obstacles):
    # YOUR CODE GOES HERE
    # Place the robot coordinates at the point
    tempRobot = []
    for i in range(0, len(robot)):
        lst = list(robot[i])
        lst[0] = lst[0] + point[0]
        lst[1] = lst[1] + point[1]
        tempRobot.append(tuple(lst))

    # See the robot is inside an obstacle or vice versa
    for polygon in obstacles:
        polyBorders = Path(polygon)
        for point in tempRobot:
            if polyBorders.contains_point(point):
                return False
    # Vice versa
    roboBorders = Path(tempRobot)
    for polygon in obstacles:
        for point in polygon:
            if roboBorders.contains_point(point):
                return False

    # Check for intersects
    for polygon in obstacles:
        for i in range(0, len(polygon)):
            curr1 = tuple(polygon[i])
            next1 = (0, 0)
            if i == (len(polygon) - 1):
                next1 = tuple(polygon[0])
            else:
                next1 = tuple(polygon[i + 1])
            # robot lines
            for k in range(0, len(tempRobot)):
                curr2 = tempRobot[k]
                next2 = (0, 0)
                if k == (len(tempRobot) - 1):
                    next2 = tempRobot[0]
                else:
                    next2 = tempRobot[k + 1]
                # Check intersection
                line1 = Path([curr1, next1])
                line2 = Path([curr2, next2])
                if line1.intersects_path(line2, filled=False):
                    return False

    # Make sure it's in the graph area
    boundaries = Path([[0.0, 0.0], [0.0, 10.0], [10.0, 10.0], [10.0, 0.0]])
    catch = True
    for point in tempRobot:
        if not boundaries.contains_point(point):
            catch = False

    if catch:
        return True

    return False


'''
Checks if the robot would collide along the path
'''


def isCollisionFreePath(robot, point1, point2, obstacles):
    tempRobot = []
    for i in range(0, len(robot)):
        lst = list(robot[i])
        tempRobot.append(lst)

    for xy in tempRobot:
        temp1 = [point1[0] + xy[0], point1[1] + xy[1]]
        temp2 = [point2[0] + xy[0], point2[1] + xy[1]]
        if not isCollisionFree([temp1, temp2], [0.0, 0.0], obstacles):
            return False

    return True


'''
Find nearest x and return distance while accounting for obstacles
'''


def nearestPointWithoutCollision(robot, points, x_new, obstacles):
    x_near = -1
    distance = 15
    if x_new in points:
        return x_near, distance

    for i, coordinates in points.items():
        distance_new = calculateDistance(x_new, points[i])
        if distance_new < distance and isCollisionFreePath(robot, x_new, points[i], obstacles):
            distance = distance_new
            x_near = i

    return x_near, distance


'''
The full RRT algorithm
'''

def RRT(robot, obstacles, startPoint, goalPoint):
    points = dict()
    tree = dict()
    path = []
    # YOUR CODE GOES HERE

    if not isCollisionFree(robot, goalPoint, obstacles):
        print("Invalid input, the end point is invalid.")
        exit(1)
    if not isCollisionFree(robot, startPoint, obstacles):
        print("Invalid input, the start point is invalid.")
        exit(1)
    points[1] = startPoint
    tree[1] = []

    if startPoint == goalPoint:
        print("Trivial solution.")
        tree[1].append(2)
        points[2] = goalPoint
        tree[2] = []
        tree[2].append(1)
    elif isCollisionFreePath(robot, startPoint, goalPoint, obstacles):
        print("Direct solution.")
        distance = calculateDistance(startPoint, goalPoint)
        tree[1].append(2)
        points[2] = goalPoint
        tree[2] = []
        tree[2].append(1)
        tree[2].append([1, distance])
    else:
        print("Generating RRT.")
        counter = 3
        while True:
            x = np.random.randint(0, 100) / 10.0
            y = np.random.randint(0, 100) / 10.0
            x_new = (x, y)
            if isCollisionFree(robot, x_new, obstacles):
                x_near, distance = nearestPointWithoutCollision(robot, points, x_new, obstacles)
                if x_near != -1:
                    points[counter] = x_new
                    tree[counter] = []
                    tree[counter].append(x_near)
                    tree[x_near].append(counter)
                    if isCollisionFreePath(robot, x_new, goalPoint, obstacles):
                        points[2] = goalPoint
                        tree[2] = []
                        tree[2].append(counter)
                        tree[counter].append(2)
                        break
                    counter = counter + 1

    # Find path
    path = basicSearch(tree, 1, 2)

    robotStart = []
    robotGoal = []
    for i in range(0, len(robot)):
        lst = list(robot[i])
        lst[0] = lst[0] + startPoint[0]
        lst[1] = lst[1] + startPoint[1]
        robotStart.append(tuple(lst))
    for i in range(0, len(robot)):
        lst = list(robot[i])
        lst[0] = lst[0] + goalPoint[0]
        lst[1] = lst[1] + goalPoint[1]
        robotGoal.append(tuple(lst))

    displayRRTandPath(points, tree, path, robotStart, robotGoal, obstacles)
    return points, tree, path


'''**********************************************************'''


def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0:
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    robotStart = var = [(x + x1, y + y1) for x, y in robot]
    robotGoal = var = [(x + x2, y + y2) for x, y in robot]

    # Visualize
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many more points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (6, 8)
    points[7] = (4.4, 2.8)
    points[8] = (8.3, 2.1)
    points[9] = (7.7, 6.3)
    points[10] = (9, 0.6)

    # points[1] = (5, 5)
    # points[2] = (7, 8.2)
    # points[3] = (6.5, 5.2)
    # points[4] = (0.3, 4)
    # points[5] = (6, 3.7)
    # points[6] = (9.7, 6.4)
    # points[7] = (4.4, 2.8)
    # points[8] = (9.1, 3.1)
    # points[9] = (8.1, 6.5)
    # points[10] = (0.7, 5.4)
    # points[11] = (5.1, 3.9)
    # points[12] = (2, 6)
    # points[13] = (0.5, 6.7)
    # points[14] = (8.3, 2.1)
    # points[15] = (7.7, 6.3)
    # points[16] = (7.9, 5)
    # points[17] = (4.8, 6.1)
    # points[18] = (3.2, 9.3)
    # points[19] = (7.3, 5.8)
    # points[20] = (9, 0.6)

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")

    newPoints, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(newPoints))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution
    # change 1 and (20 or 10) as you want

    path = basicSearch(adjListMap, 1, 10)
    # path = basicSearch(adjListMap, 1, 20)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code
    if display == 'display':
        displayRRTandPath(newPoints, adjListMap, path)

        # Solve a real RRT problem
    newPoints, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))

    # Your visualization code
    if display == 'display':
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)


if __name__ == "__main__":
    # Retrive file name for input data
    if len(sys.argv) < 6:
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if len(sys.argv) == 7:
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)
