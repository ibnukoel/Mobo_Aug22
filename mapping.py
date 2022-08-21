
map = [['#', ' ', ' ', ' '],
       [' ', ' ', ' ', ' '],
       [' ', ' ', ' ', ' '],
       [' ', ' ', ' ', ' ']]

valnya = [[1, 2, 4, 7],
          [99, 3, 6, 11],
          [10, 5, 9, 14],
          [13, 8, 12, 15]]
          
grid = [[0, 0, 99, 0],
        [99, 0, 99, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]]
grid_scale = 12.5

delta = [[-1, 0, 1],  # go up
         [0, -1, 1],  # go left
         [1, 0, 1],  # go down
         [0, 1, 1]]  # go right

#next move,robot facing, x , y ,left,right)
movementRobot = [[2, 0,    0, 1,  0, 1],
                 [4, -90,  1, 1,  1, 0],
                 [6, 0,    1, 2,  0, 1],
                 [11, 0,   2, 1,  0, 1],
                 [14, -90, 2, 3,  1, 0],
                 [15, -90, 3, 3,  1, 0]]
                 
def printmap(grid):
    for i in range(len(grid)):
        print(grid[i])
