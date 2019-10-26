import numpy as np
import matplotlib.pyplot as plt

##########################################
########     initialization       ########
##########################################
mapmap = np.load('map.npy')
# mapmap = np.zeros((15,15),dtype=np.int)
print(mapmap.shape)
startPosition = (2, 0)  # Initial point
goalPosition = (13, 11)  # End point
direction = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]  # reachable direction
mapRow, mapCol = mapmap.shape

# visualization
mapViz = mapmap.copy()
mapViz[startPosition[0], startPosition[1]] = 20
mapViz[goalPosition[0], goalPosition[1]] = 30
plt.pcolormesh(mapViz)
plt.show()

dx = abs(startPosition[0] - goalPosition[0])
dy = abs(startPosition[1] - goalPosition[1])
cost = np.sqrt(dx ^ 2 + dy ^ 2)


##########################################
########    heuristic function    ########
##########################################
def g(current, node):
    return current[3] + abs(node[0] - current[0]) + abs(node[1] - current[1])


def h(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


def f(parameters):
    # code here ...
    pass


##########################################
########     other function       ########
##########################################
# It's optional, you can write any function useful for you here.
# You can also write nothing.


##########################################
########       A* algorithm       ########
##########################################
# Advertised format of elements in openList and closeList is
# tuple(x,y,f,g,h,parentNodeIdx)
# x: row of current node
# y: column of current node
# f,g,h: f,g,h in A* algorithm, f=g+h
# parentNodeIdx: index of current node's parent node in closeList 
# ----------------------------------------
# You can change these, but remember code for visualization 
# should also be modefied.

# initialization
openList = []
closeList = []

openList.append(startPosition + (0, 0, 0, -1))

while openList:
    current = min(openList, key=lambda t: t[2])
    openList.pop(openList.index(current))
    closeList.append(current)
    currentIdx = len(closeList) - 1
    if current[0:2] == goalPosition:
        break
    for d in direction:
        neighbor = (current[0] + d[0], current[1] + d[1])
        if neighbor[0] >= mapRow or neighbor[0] < 0 or neighbor[1] >= mapCol or neighbor[1] < 0:
            continue

        if mapmap[neighbor]:
            continue

        if any([node[0:2] == neighbor for node in closeList]):
            continue

        g_score = g(current, neighbor)
        h_score = h(neighbor, goalPosition)
        neighbor += ((g_score + h_score) * 2, g_score, h_score, currentIdx)
        l = list(filter(lambda node: node[0:2] == neighbor[0:2], openList))
        if not len(l):
            openList.append(neighbor)
        elif l[0][3] > g_score:
            idx = openList.index(l[0])
            openList.pop(idx)
            openList.append(neighbor)

##########################################
########      visualization       ########
##########################################
fig, (ax0, ax1) = plt.subplots(1, 2)
fig.suptitle('A* algorithm', fontsize=36)
ax0.set_title('Searching process')
ax1.set_title('Searched path')

mapViz = mapmap.copy()
mapViz[startPosition[0], startPosition[1]] = 30
mapViz[goalPosition[0], goalPosition[1]] = 30
for k in range(len(closeList) - 1):
    mapViz[closeList[k][0], closeList[k][1]] = 200 - closeList[k][2]
    ax0.pcolormesh(mapViz)
    plt.pause(0.1)

mapViz = mapmap.copy()
mapViz[startPosition[0], startPosition[1]] = 30
mapViz[goalPosition[0], goalPosition[1]] = 30
path = []
nodeIdx = -1
while True:
    if closeList[nodeIdx][0:2] == startPosition:
        break
    path.append(closeList[nodeIdx][0:2])
    nodeIdx = closeList[nodeIdx][-1]
path.reverse()
for k in range(len(path) - 1):
    mapViz[path[k][0], path[k][1]] = 20
    ax1.pcolormesh(mapViz)
    plt.pause(0.1)

plt.show()
