def findPath(mapmap, startPosition, goalPosition):
    ##########################################
    ########     initialization       ########
    ##########################################
    direction = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]  # reachable direction
    mapRow, mapCol = mapmap.shape

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

    path = []
    nodeIdx = -1
    while True:
        if closeList[nodeIdx][0:2] == startPosition:
            break
        path.append(closeList[nodeIdx][0:2])
        nodeIdx = closeList[nodeIdx][-1]
    path.reverse()

    return path
