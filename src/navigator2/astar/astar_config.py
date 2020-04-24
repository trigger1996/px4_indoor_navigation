#encoding=utf-8

def manhattan(p,p_end):
    return 10*(abs(p[0]-p_end[0])+abs(p[1]-p_end[1])+abs(p[2]-p_end[2]))

def square(p, p_end):
    return abs(p[0]-p_end[0])**2 + abs(p[1]-p_end[1])**2 + abs(p[2]-p_end[2])**2

def manhattan_2d(p,p_end):
    return 10*(abs(p[0]-p_end[0])+abs(p[1]-p_end[1]))

def square_2d(p, p_end):
    return (p[0]-p_end[0])**2 + (p[1]-p_end[1])**2

# astar_config={
#     'movement_list': [(1, 0, 0),
#                       (-1, 0, 0),
#                       (0, 1, 0),
#                       (0, -1, 0),
#                       (0, 0, 1),
#                       (0, 0, -1)],
#
#     'func_h': manhattan,
#     'z_move_cost': 1
# }


# step_size = 1
# astar_config={
#    'movement_list': [(step_size, 0, 0),
#                      (-step_size, 0, 0),
#                      (0, step_size, 0),
#                      (0, -step_size, 0),
#                      (0, 0, step_size),
#                      (0, 0, -step_size)],
#
#     'func_h': square,
#     'z_move_cost': 5
# }

step_size = 1       # in grids
astar_config={
    'movement_list': [(step_size,   0,          0),
                      (-step_size,  0,          0),
                      (0,           step_size,  0),
                      (0,          -step_size,  0),
#                     (step_size,   step_size,  0),
#                     (step_size,  -step_size,  0),
#                     (-step_size,  step_size,  0),
#                     (-step_size, -step_size,  0),
                      (0,           0,          step_size),
                      (0,           0,         -step_size)],

    'movement_list_2d': [(step_size,      0),
                         (-step_size,     0),
                         (0,              step_size),
                         (0,             -step_size),
                         (step_size,      step_size),
                         (step_size,     -step_size),
                         (-step_size,     step_size),
                         (-step_size,    -step_size)],

    'func_h':    square,
    'func_h_2d': square_2d,
    'z_move_cost': 35
}
