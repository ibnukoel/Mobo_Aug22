# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
#
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left,
# up, and down motions. Note that the 'v' should be
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------


#garis di atas = 98,+kanan = 94, +kiri = 93,  +kanan+kiri = 90
#garis di kanan = 97,+kiri = 86               +atas+bawah = 89
#garis di bawah = 96,+kanan = 92, +kiri = 91, +kanan+kiri = 88
#garis di kiri = 95,                          +atas+bawah = 87


mapLine = [[93, 98, 98, 98, 98, 94],
        [91, 96, 96, 96, 96, 97],
        [1, 1, 1, 1, 1, 86],
        [0, 0, 0, 0, 1, 86],
        [0, 0, 1, 0, 1, 88]]

grid = [[0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [0, -1], # go left
         [1, 0], # go down
         [0, 1]] # go right

delta_val = ['-90', '180', '90', '0']
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

    #creat path from action / direction
    x = goal[0]
    y = goal[1]
    expand[x][y]="*"
    while x !=init[0] or y!=init[1]:
        # movement configuration :
        # urutan,next move,robot facing, x , y , deltaRho1, deltaRho2,rhomin)
        #movement = [0, z, x, y, 0, 0, 0]
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        expand[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2

    return action,expand #return expand # make sure you return the shortest path

action,newpath = search(grid,init,goal,cost)
printmap(newpath)
printmap(action)

