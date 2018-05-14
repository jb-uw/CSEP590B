#!/usr/bin/env python3

def adjust_goal_coord(goal_x, goal_y, angle_z):
    if angle_z >= -45 and \
       angle_z < 45:
        goal_x -= 4
        goal_angle = 0
    elif angle_z >= 45 and \
         angle_z < 135:
        goal_y -= 4
        goal_angle = 90
    elif angle_z >= 135 or \
         angle_z < -135:
        goal_x += 4
        goal_angle = 180
    elif angle_z >= -135 and \
         angle_z < -45:
        goal_y += 4
        goal_angle = 270

    return (goal_x, goal_y), goal_angle
    
def get_obstacles_around_coord(x, y):
    return [(x+3, y  ), (x+3, y+1), (x+3, y+2), (x+3, y+3), \
            (x+2, y+3), (x+1, y+3), (x  , y+3), (x-1, y+3), \
            (x-2, y+3), (x-3, y+3), (x-3, y+2), (x-3, y+1), \
            (x-3, y  ), (x-3, y-1), (x-3, y-2), (x-3, y-3), \
            (x-2, y-3), (x-1, y-3), (x  , y-3), (x+1, y-3), \
            (x+2, y-3), (x+3, y-3), (x+3, y-2), (x+3, y-1)]
                       
def get_obstacles_coords(x, y):
    return [(x  , y  ), (x+1, y  ), (x+1, y+1), \
            (x  , y+1), (x-1, y+1), (x-1, y  ), \
            (x-1, y-1), (x  , y-1), (x+1, y-1)]
            
def get_step_angle(current_grid, next_grid, current_angle):
    dx = next_grid[0] - current_grid[0]
    dy = next_grid[1] - current_grid[1]
    if dx == 1 and \
       dy == 0:
        angle = 0 - current_angle
    elif dx == 1 and \
         dy == 1:
        angle = 45 - current_angle
    elif dx == 0 and \
         dy == 1:
        angle = 90 - current_angle
    elif dx == -1 and \
         dy == 1:
        angle = 135 - current_angle
    elif dx == -1 and \
         dy == 0:
        angle = 180 - current_angle
    elif dx == -1 and \
         dy == -1:
        angle = 225 - current_angle
    elif dx == 0 and \
         dy == -1:
        angle = 270 - current_angle
    elif dx == 1 and \
         dy == -1:
        angle = 315 - current_angle
    if angle > 180:
        angle = angle - 360
    return angle