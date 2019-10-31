import math

def findPath(mapmap, startPosition, goalPosition):
    ##########################################
    ########     initialization       ########
    ##########################################
    direction = [(1, 1), (1, -1), (-1, 1), (-1, -1), (0, 1), (0, -1), (1, 0), (-1, 0)]  # reachable direction
    mapRow, mapCol = mapmap.shape

    ##########################################
    ########    heuristic function    ########
    ##########################################
    coef = 2 - math.sqrt(2)

    def g(current, node):
        # octile distance
        dx = abs(node[0] - current[0])
        dy = abs(node[1] - current[1])
        return current[3] + dx + dy - coef * min(dx, dy)

    def h(node, goal):
        # octile heuristic
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        return dx + dy - coef * min(dx, dy)

    def f(g_score, h_score):
        return g_score + h_score

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
    # should also be modified.

    # initialization
    openList = []
    closeList = []

    # put startPosition in openList
    g_score = 0
    h_score = h(startPosition, goalPosition)
    openList.append(startPosition + (f(g_score, h_score), g_score, h_score, -1))

    while openList:
        # find node with smallest f and its index in openList, add it to closeList and pop from openList
        index, current = min(enumerate(openList), key=lambda t: t[1][2])
        openList.pop(index)
        closeList.append(current)
        currentIdx = len(closeList) - 1

        if current[0:2] == goalPosition:
            break  # stop if found goalPosition

        for d in direction:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if neighbor[0] >= mapRow or neighbor[0] < 0 or neighbor[1] >= mapCol or neighbor[1] < 0:
                continue  # out of range

            if mapmap[neighbor]:
                continue  # obstacle

            if any([node[0:2] == neighbor for node in closeList]):
                continue  # already in closeList

            # compute g_score and h_score, add to openList
            g_score = g(current, neighbor)
            h_score = h(neighbor, goalPosition)
            neighbor += (f(g_score, h_score), g_score, h_score, currentIdx)
            # find nodes in openList with current position
            nodes = list(filter(lambda node: node[0:2] == neighbor[0:2], openList))
            if not len(nodes):
                openList.append(neighbor)
            elif nodes[0][3] > g_score:
                # update existing nodes if g_score is smaller
                idx = openList.index(nodes[0])
                openList.pop(idx)
                openList.append(neighbor)

    path = []
    nodeIdx = -1
    while True:
        if closeList[nodeIdx][0:2] == startPosition:
            break
        path.append(closeList[nodeIdx][0:2])
        nodeIdx = closeList[nodeIdx][-1]
    path.reverse()

    return path
