
#garis di atas = 98,+kanan = 94, +kiri = 93,  +kanan+kiri = 90
#garis di kanan = 97,+kiri = 86               +atas+bawah = 89
#garis di bawah = 96,+kanan = 92, +kiri = 91, +kanan+kiri = 88
#garis di kiri = 95,                          +atas+bawah = 87


mapLine = [[93, 98, 98, 98, 94],
        [91, 96, 96, 96, 97],
        [0, 0, 0, 1, 88]]

grid = [[0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
goal_dir = 0
cost = 1
dp1 = 14
dp2 = 2

delta = [[-1, 0], # go up
         [0, -1], # go left
         [1, 0], # go down
         [0, 1]] # go right

delta_val = [-90, 180, 90, 0]
delta_name = ['^', '<', 'v', '>']

def printmap(grid):
    for i in range(len(grid)):
        print(grid[i])

def search(grid,init,goal,cost):
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    # action is direction from goal to initial location

    closed[init[0]][init[1]] = 1

    x = init[0]
    y = init[1]
    g = 1

    open = [[g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            #resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g = g + cost
                            open.append([g, x2, y2])
                            closed[x2][y2] = g
                            action[x2][y2] = i
                            #g = g2


    #path ilustration
    #creat path from action / direction
    x = goal[0]
    y = goal[1]
    expand[x][y]="*"
    while x !=init[0] or y!=init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        expand[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2
    printmap(expand)
    return action #return expand # make sure you return the shortest path

action = search(grid,init,goal,cost)
#printmap(newpath)
#printmap(action)

def deltaRho_cal(x,y,z,dx,dy):
    ML = mapLine[x][y]
    x_2 = x + dx
    y_2 = y + dy
    # garis di atas = 98,+kanan = 94, +kiri = 93,  +kanan+kiri = 90
    # garis di kanan = 97,+kiri = 86               +atas+bawah = 89
    # garis di bawah = 96,+kanan = 92, +kiri = 91, +kanan+kiri = 88
    # garis di kiri = 95,                          +atas+bawah = 87
    if z == 0:
        #if ML == 98 or ML == 96 or ML == 95 or ML == 93 or ML == 91 or ML == 87:
        if ML == 97 or ML == 94 or ML == 92 or ML == 90 or ML == 89 or ML == 86 or ML == 88:
            dp = dp1
        else: dp = 0
    elif z == 90:
        if ML == 89 or ML == 96 or ML == 92 or ML == 91 or ML == 88 or ML == 87:
            dp = dp1
        else: dp = 0
    elif z == 180:
        if ML == 93 or ML == 90 or ML == 86 or ML == 91 or ML == 95 or ML == 87 or ML == 88:
            dp = dp1
        else: dp = 0
    elif z == -90:
        if ML == 98 or ML == 90 or ML == 94 or ML == 93 or ML == 89 or ML == 87:
            dp = dp1
        else: dp = 0


    #print(ML,x,y,dp,z)

    if dp == 0 and x_2 < len(mapLine) and y_2 < len(mapLine[1]):
        ML = mapLine[x_2][y_2]
        # garis di atas = 98,+kanan = 94, +kiri = 93,  +kanan+kiri = 90
        # garis di kanan = 97,+kiri = 86               +atas+bawah = 89
        # garis di bawah = 96,+kanan = 92, +kiri = 91, +kanan+kiri = 88
        # garis di kiri = 95,                          +atas+bawah = 87
        if z == 0:
            # if ML == 98 or ML == 96 or ML == 95 or ML == 93 or ML == 91 or ML == 87:
            if ML == 97 or ML == 94 or ML == 92 or ML == 90 or ML == 89 or ML == 86 or ML == 88:
                dp = dp2
            else:
                dp = 0
        elif z == 90:
            if ML == 89 or ML == 96 or ML == 92 or ML == 91 or ML == 88 or ML == 87:
                dp = dp2
            else:
                dp = 0
        elif z == 180:
            if ML == 93 or ML == 90 or ML == 86 or ML == 91 or ML == 95 or ML == 87 or ML == 88:
                dp = dp2
            else:
                dp = 0
        elif z == -90:
            if ML == 98 or ML == 90 or ML == 94 or ML == 93 or ML == 89 or ML == 87:
                dp = dp2
            else:
                dp = 0
        #print("kot2", ML, x_2, y_2, dp,z)

    return dp

def movement_path():
    # movement configuration :
    # urutan,next move,robot facing, x , y , deltaRho1, deltaRho2,rhomin)
    # movement = [0, z, x, y, 0, 0, 0]
    x = goal[0]
    y = goal[1]
    z2 = goal_dir
    z1 = delta_val[action[x][y]]
    dx=0
    dy=0
    dpA = deltaRho_cal(x, y, z1,dx,dy)
    dpB = deltaRho_cal(x, y, z2,dx,dy)
    movement = [[0, z2, x, y, dpA, dpB]]
    i = 1
    #print(i, z2, x, y, dpA, dpB)
    #print("===")

    while x !=init[0] or y !=init[1]:
        dx = delta[action[x][y]][0]
        dy = delta[action[x][y]][1]
        x2 = x - dx
        y2 = y - dy
        z2 = z1
        z1 = delta_val[action[x2][y2]]
        dpA = deltaRho_cal(x2, y2, z1,dx,dy)
        dpB = deltaRho_cal(x2, y2, z2,dx,dy)
        #print(i,z2,x2,y2,dpA,dpB)
        #print("===")
        movement.append([i, z2, x2, y2, dpA, dpB])
        x = x2
        y = y2
        i = i+1
    movement.sort(reverse=True)
    return movement



print("=====")
movement = movement_path()
printmap(movement)
