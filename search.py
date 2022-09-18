import math, numpy, random
grid = [[0, 0, 99, 0],
        [99, 0, 0, 0],
        [0, 99, 0, 0],
        [0, 0, 0, 0]]
con = 1
map = [['#', ' ', ' ', ' '],
       [' ', ' ', ' ', ' '],
       [' ', ' ', ' ', ' '],
       [' ', ' ', ' ', ' ']]
visited = grid
init = [0, 0]
goal = [3, 3]
hitung = 0
delta = [[-1, 0, 1,90],  # go up
         [0, -1, 1,180],  # go left
         [1, 0, 1,-90],  # go down
         [0, 1, 1,0]]  # go right
movementRobot = [[2, 0, 0, 1, 0, 1],
                 [4, 0, 0, 2, 0, 1],
                 [7, 0, 0, 3, 0, 1],
                 [11, -90, 1, 3, 1, 0],
                 [14, -90, 2, 3, 1, 0],
                 [15, -90, 3, 3, 1, 0]]

def printmap(grid):
    for i in range(len(grid)):
        print(grid[i])

def search(grid, init, goal, hitung):
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0

    open = [[g, x, y]]
    #print("open ",open)
    found = False
    resign = False
    count = 0

    while not found and not resign:
        open.sort()
        open.reverse()
        next = open.pop()
        #print(next)
        x = next[1]
        y = next[2]
        z = next[0]
        # print(open)
        expand[x][y] = count
        count += 1

        if x == goal[0] and y == goal[1]:
            found = True
        else:
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                # print(delta[i][3])
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        g2 = delta[i][2]
                        open.append([g2, x2, y2])
                        closed[x2][y2] = 1
                        visited[x2][y2] = hitung
                        hitung += 1
                        #for i in range(len(visited)):
                            #print(closed[i], "    ", visited[i])
    return expand,hitung
    # print("++++++++++++++++")

"""
def nextmovenya(bobot, valnya, delta):
    x = init[0]
    y = init[1]
    bobotcomparison = []
    for i in range(len(delta)):
        x2 = x + delta[i][0]
        y2 = y + delta[i][1]
        # print('valnya',valnya[x2][y2])
        if x2 >= 0 and x2 < len(valnya) and y2 >= 0 and y2 < len(valnya[0]):
            bobotcomparison.append([bobot[0][i] * valnya[x2][y2], i])
    print(bobotcomparison)
    zz = numpy.transpose(bobotcomparison)
    print('bobotcom.t', zz)
    b_min = min(zz[0])
    print('min', b_min)

    print('bobotcom', bobotcomparison)
    for i in range(len(bobotcomparison)):
        if (b_min == bobotcomparison[i][0]):
            nextmove = bobotcomparison[i][1]
    # dapat next move
    nextmove2 = bobot[1][nextmove]
    return nextmove2
"""

def nextmovement(grid,init,goal):
    bobot = [0]
    #closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    x = init[0]
    y = init[1]
    x3 = x
    y3 = y

    bobot[0] = [grid[init[0]][init[1]], x, y]
    step = 0
    #print(bobot)
    found = False
    resign = False
    flaginput = 0

    while not found and not resign:
        if step == 100:
            found = True
        if x == goal[0] and y == goal[1]:
            found = True
        else:
            for i in range(len(delta)):
                #print("delta")
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]

                # print(delta[i][3])
                if x2>=0 and y2>=0 and x2 < len(grid) and y2 < len(grid[0]):
                    #print("masih di grid")
                    #print("x2 =",x2 ,", y2 =", y2)
                    print("============================")
                    print("[",step,"]",x,y," arah ke",i,"--",grid[x][y],"vs",grid[x2][y2])
                    if grid[x2][y2] >= grid[x3][y3]:
                        bobot.append(0)
                        if flaginput == 0:
                            bobot[0] = [grid[x2][y2], delta[i][3], x2, y2]
                            flaginput = 1
                        bobot[step] = [grid[x2][y2],delta[i][3],x2,y2]
                        print(bobot)
                        x3 = x2
                        y3 = y2
                        print(x2, y2, "add")
                    else:
                        print(x2, y2, "remove")
                #else :
                #    print("out grid")
            x = x3
            y = y3
            step = step + 1
            print("move",step)
    return bobot
    # print("++++++++++++++++")


map_baru = search(grid, init, goal, hitung)[0]

print(nextmovement(map_baru,init,goal))
#print(movementRobot)
printmap(map_baru)

#arah = move(map_baru,init,goal,hitung)
#printmap(arah)
