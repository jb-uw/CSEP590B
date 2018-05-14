
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
    obstacle_cubes = [None, None]
    path = None
    grid_frame_pose = Pose(robot.pose.position.x, robot.pose.position.y, 0, angle_z = robot.pose.rotation.angle_z)
    current_grid = (0, 0)
    state = 'INIT'
    action = 'NONE'
    current_angle = 0
    
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    
    while not stopevent.is_set():
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=0.1, include_existing=True)
            if cube.cube_id == 1:
                action = 'GOAL_CUBE_SEEN'
                print("Goal Cube seen")
            if cube.cube_id > 1:
                action = 'OBSTACLE_CUBE_SEEN'
                print("Obstacle Cube seen")
            cube_relative_pose = get_relative_pose(cube.pose, grid_frame_pose)
            coord_x = math.floor((cube_relative_pose.position.x+25+12) / grid.scale)
            coord_y = math.floor((cube_relative_pose.position.y+12) / grid.scale)
        except asyncio.TimeoutError:
            print("NO_CUBE_SEEN")
            action = 'No Cube seen'
            
        if state == 'INIT':
            if action == 'GOAL_CUBE_SEEN':
                print("INIT - GOAL_CUBE_SEEN")
                # setup goal and calculate path
                grid.clearGoals()
                goal_angle = cube_relative_pose.rotation.angle_z.degrees
                adjusted_goal = adjust_goal_coord(coord_x, coord_y, goal_angle)
                goal_grid = adjusted_goal[0]
                goal_angle = adjusted_goal[1]
                grid.addGoal(goal_grid)
                obstacles = get_obstacles_around_coord(coord_x, coord_y)
                grid.addObstacles(obstacles)
                state = 'FOUND_GOAL'
            elif action == 'OBSTACLE_CUBE_SEEN':
                print("INIT - OBSTACLE_CUBE_SEEN")
                if cube.cube_id == 2:
                    obstacle_id = 0
                else:
                    obstacle_id = 1
                if obstacle_cubes[obstacle_id] == None: # new obstacle found
                    print("Add Obstacle")
                    # mark it as seen
                    obstacle_cubes[obstacle_id] = (coord_x, coord_y)
                    # add obstacle to grid
                    obstacles = get_obstacles_around_coord(coord_x, coord_y)
                    grid.addObstacles(obstacles)
                    state = 'NO_GOAL'
            elif action == 'NO_CUBE_SEEN':
                print("INIT - NO_CUBE_SEEN")
                state = 'NO_GOAL'
            path = calculate_path(current_grid)
                
        elif state == 'NO_GOAL':
            if action == 'GOAL_CUBE_SEEN':
                print("NO_GOAL - GOAL_CUBE_SEEN")
                # setup goal and calculate path
                grid.clearGoals()
                goal_angle = cube_relative_pose.rotation.angle_z.degrees
                adjusted_goal = adjust_goal_coord(coord_x, coord_y, goal_angle)
                goal_grid = adjusted_goal[0]
                goal_angle = adjusted_goal[1]
                grid.addGoal(goal_grid)
                obstacles = get_obstacles_around_coord(coord_x, coord_y)
                grid.addObstacles(obstacles)
                path = calculate_path(current_grid)
                state = 'FOUND_GOAL'
            elif action == 'OBSTACLE_CUBE_SEEN':
                print("NO_GOAL - OBSTACLE_CUBE_SEEN")
                if cube.cube_id == 2:
                    obstacle_id = 0
                else:
                    obstacle_id = 1
                if obstacle_cubes[obstacle_id] == None: # new obstacle found
                    print("Add Obstacle")
                    # mark it as seen
                    obstacle_cubes[obstacle_id] = (coord_x, coord_y)
                    # add obstacle to grid
                    obstacles = get_obstacles_around_coord(coord_x, coord_y)
                    grid.addObstacles(obstacles)
                    path = calculate_path(current_grid)
            elif action == 'NO_CUBE_SEEN':
                print("NO_GOAL - NO_CUBE_SEEN")
                    
        elif state == 'FOUND_GOAL':
            if action == 'GOAL_CUBE_SEEN':
                print("FOUND_GOAL - GOAL_CUBE_SEEN")
            elif action == 'OBSTACLE_CUBE_SEEN':
                print("FOUND_GOAL - OBSTACLE_CUBE_SEEN")
                if cube.cube_id == 2:
                    obstacle_id = 0
                else:
                    obstacle_id = 1
                if obstacle_cubes[obstacle_id] == None: # new obstacle found
                    print("Add Obstacle")
                    # mark it as seen
                    obstacle_cubes[obstacle_id] = (coord_x, coord_y)
                    # add obstacle to grid
                    obstacles = get_obstacles_around_coord(coord_x, coord_y)
                    grid.addObstacles(obstacles)
                    path = calculate_path(current_grid)
            elif action == 'NO_CUBE_SEEN':
                print("FOUND_GOAL - NO_CUBE_SEEN")
        
        elif state == 'TURN_IN_PLACE':
            if action == 'GOAL_CUBE_SEEN':
                print("TURN_IN_PLACE - GOAL_CUBE_SEEN")
                # recalibrate current position
                robot_relative_pose = get_relative_pose(robot.pose, grid_frame_pose)
                robot_x = math.floor((robot_relative_pose.position.x+12) / grid.scale)
                robot_y = math.floor((robot_relative_pose.position.y+12) / grid.scale)
                current_grid = (robot_x, robot_y)
                current_angle = robot_relative_pose.rotation.angle_z.degrees
                # setup goal and calculate path
                grid.clearGoals()
                goal_angle = cube_relative_pose.rotation.angle_z.degrees
                adjusted_goal = adjust_goal_coord(coord_x, coord_y, goal_angle)
                goal_grid = adjusted_goal[0]
                goal_angle = adjusted_goal[1]
                grid.addGoal(goal_grid)
                obstacles = get_obstacles_around_coord(coord_x, coord_y)
                grid.addObstacles(obstacles)
                path = calculate_path(current_grid)
                state = 'FOUND_GOAL'
            elif action == 'OBSTACLE_CUBE_SEEN':
                print("TURN_IN_PLACE - OBSTACLE_CUBE_SEEN")
                if cube.cube_id == 2:
                    obstacle_id = 0
                else:
                    obstacle_id = 1
                if obstacle_cubes[obstacle_id] == None: # new obstacle found
                    print("Add Obstacle")
                    # mark it as seen
                    obstacle_cubes[obstacle_id] = (coord_x, coord_y)
                    # add obstacle to grid
                    obstacles = get_obstacles_around_coord(coord_x, coord_y)
                    grid.addObstacles(obstacles)
                robot.turn_in_place(degrees(10), speed=degrees(20), in_parallel=False).wait_for_completed()
            elif action == 'NO_CUBE_SEEN':
                print("TURN_IN_PLACE - NO_CUBE_SEEN")
                robot.turn_in_place(degrees(10), speed=degrees(20), in_parallel=False).wait_for_completed()
            
        if len(path) > 0:
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

