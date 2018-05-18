
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from searchtree import *
import asyncio
from pose_transform import *
from utils import *
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps, radians

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """   
    tree = SearchTree(grid, heuristic)
    path = tree.find_goal()
    grid.setPath(path)
    return path


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    return current.weight + math.sqrt((goal[0]-current.coord[0])**2 + (goal[1] - current.coord[1])**2)

def calculate_path(current_grid):
    grid.clearVisited()
    grid.setStart(current_grid)
    path = astar(grid, heuristic)
    path.pop(0)
    return path
    
def update_grid_with_goal(goal_x, goal_y, goal_angle):
    grid.clearGoals()
    adjusted_goal = adjust_goal_coord(goal_x, goal_y, goal_angle)
    goal_grid = adjusted_goal[0]
    goal_angle = adjusted_goal[1]
    grid.addGoal(goal_grid)
    obstacles = get_obstacles_around_coord(goal_x, goal_y)
    grid.addObstacles(obstacles)
    return goal_angle
    
def get_relative_coord(cube, reference_pose):
    cube_relative_pose = get_relative_pose(cube.pose, reference_pose)
    coord_x = math.floor((cube_relative_pose.position.x+25+12) / grid.scale)
    coord_y = math.floor((cube_relative_pose.position.y+12) / grid.scale)
    return coord_x, coord_y, cube_relative_pose.rotation.angle_z.degrees

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    center = (grid.width/2, grid.height/2)
    goal_grid = center
    grid.addGoal(center)
    goal_angle = 0
    path = None
    grid_frame_pose = Pose(robot.pose.position.x, robot.pose.position.y, 0, angle_z = robot.pose.rotation.angle_z)
    current_grid = (0, 0)
    state = 'INIT'
    current_angle = 0
    cube2_seen = False
    cube3_seen = False
    
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    
    while not stopevent.is_set():
        cube1 = robot.world.get_light_cube(1)
        cube2 = robot.world.get_light_cube(2)
        cube3 = robot.world.get_light_cube(3)

        if cube1.is_visible:
            if state == 'INIT' or state == 'NO_GOAL':
                print('Cube 1 seen, add and go to goal')
                # setup goal and calculate path
                coord_x, coord_y, goal_angle = get_relative_coord(cube1, grid_frame_pose)
                goal_angle = update_grid_with_goal(coord_x, coord_y, goal_angle)
                path = calculate_path(current_grid)
                state = 'FOUND_GOAL'
            elif state == 'TURN_IN_PLACE':
                print('TURNING: Cube 1 seen, add and go to goal')
                # recalibrate current position
                robot_x, robot_y, robot_angle = get_relative_coord(robot, grid_frame_pose)
                current_grid = (robot_x, robot_y)
                current_angle = robot_angle
                # setup goal and calculate path
                coord_x, coord_y, goal_angle = get_relative_coord(cube1, grid_frame_pose)
                goal_angle = update_grid_with_goal(coord_x, coord_y, goal_angle)
                path = calculate_path(current_grid)
                state = 'FOUND_GOAL'
            
        if cube2.is_visible:
            if state == 'INIT':
                state = 'NO_GOAL'
            print("Cube 2 seen")
            if not cube2_seen:
                # mark it as seen
                cube2_seen = True
                # add obstacle to grid
                coord_x, coord_y, goal_angle = get_relative_coord(cube2, grid_frame_pose)
                obstacles = get_obstacles_around_coord(coord_x, coord_y)
                grid.addObstacles(obstacles)
            if state != 'TURN_IN_PLACE':
                path = calculate_path(current_grid)
            else:
                robot.turn_in_place(degrees(10), speed=degrees(20), in_parallel=False).wait_for_completed()
                
        if cube3.is_visible:
            if state == 'INIT':
                state = 'NO_GOAL'
            print("Cube 3 seen")
            if not cube3_seen:
                # mark it as seen
                cube3_seen = True
                # add obstacle to grid
                coord_x, coord_y, goal_angle = get_relative_coord(cube3, grid_frame_pose)
                obstacles = get_obstacles_around_coord(coord_x, coord_y)
                grid.addObstacles(obstacles)
            if state != 'TURN_IN_PLACE':
                path = calculate_path(current_grid)
            else:
                robot.turn_in_place(degrees(10), speed=degrees(20), in_parallel=False).wait_for_completed()

        if not (cube1.is_visible or cube2.is_visible or cube3.is_visible):
            if state == 'INIT':
                path = calculate_path(current_grid)
                state = 'NO_GOAL'
            if state == 'TURN_IN_PLACE':
                robot.turn_in_place(degrees(10), speed=degrees(20), in_parallel=False).wait_for_completed()
            
        if path is not None and len(path) > 0:
            # figure out the pose angle. should keep the angle of the path traveled
            next_grid = path.pop(0)
            angle = get_step_angle(current_grid, next_grid, current_angle)
            robot.turn_in_place(degrees(angle), speed=degrees(45), is_absolute=False).wait_for_completed()
            time.sleep(0.2)
            current_angle += angle
            if round(current_angle) % 90 == 0:
                dist = grid.scale
            else:
                dist = grid.scale*1.4142
            robot.drive_straight(distance_mm(dist), speed_mmps(25)).wait_for_completed()
            time.sleep(0.2)
            current_grid = next_grid
            grid.setStart(current_grid)
            if len(path) == 0:
                if state == 'FOUND_GOAL':
                    robot.turn_in_place(degrees(goal_angle), speed=degrees(45), is_absolute=True).wait_for_completed()
                    break
                else:
                    state = 'TURN_IN_PLACE'


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

