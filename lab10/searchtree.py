
#author1:
#author2:

from grid import *
import math
import planning

class TreeNode():

    # data members
    children = "children of this tree node"
    parent = "parent of this tree node"
    coord = "coordinate (x, y)"
    weight = "cost to get here"
    
    def __init__(self, x, y, weight, parent):
        self.parent = parent
        self.children = []
        self.coord = (x, y)
        self.weight = weight
        
        
    def __repr__(self):
        return "(x = %i, y = %i, w = %0.1f)" % (self.coord[0], self.coord[1], self.weight)
    
    def add_child(self, x, y, weight):
        self.children.append(TreeNode(x, y, weight, self))
		
    def remove_child(self, x, y):
        for child in self.children:
            if child.coord == (x , y):
                self.children.remove(child)
                return	

    def add_new_neighbors(self, grid, node_list):
        neighbors = grid.getNeighbors(self.coord)
        visited_list = grid.getVisited()
        for neighbor in neighbors:
            exist = False
            visited = False
            for node in node_list:
                if neighbor[0] == node.coord: # see if it already exist in our search list
                    if neighbor[1]+self.weight < node.weight: # remove the one with higher weight if it is already there
                        node_list.remove(node)
                        self.add_child(neighbor[0][0], neighbor[0][1], neighbor[1]+self.weight)
                        node.parent.remove_child(node.coord[0], node.coord[1])
                    exist = True
            for visited_node in visited_list:
                if neighbor[0] == visited_node: # already visited this neighbor
                    visited = True
                    break
            if not exist and not visited: # neighbor not in visited and does not exist in our search list
                self.add_child(neighbor[0][0], neighbor[0][1], neighbor[1]+self.weight)
                     
class SearchTree():

    # data members
    root = "root node of this tree"
    grid = "grid"
    heuristic = "heuristic"
    
    def __init__(self, grid, heuristic):
        start = grid.getStart()
        self.root = TreeNode(start[0], start[1], 0, None)
        self.grid = grid
        self.grid.addVisited(start)
        self.heuristic = heuristic
        
    def find_goal(self):
        goal = (self.grid.getGoals())[0]
        node_list = []
        node_list.append(self.root)
        dest = None
        
        while dest is None:
            best_weight = 99999;
            best_node = None
            # search for goal in node_list or the best node
            for node in node_list:
                if node.coord[0] == goal[0] and node.coord[1] == goal[1]:
                    dest = node
                else:
                    if best_node == None or \
                       self.heuristic(node,goal) < self.heuristic(best_node,goal):
                        best_weight = node.weight
                        best_node = node

            # search from the best node
            if dest is None:
                self.grid.addVisited(best_node.coord)
                node_list.remove(best_node)
                best_node.add_new_neighbors(self.grid, node_list)
                node_list.extend(best_node.children)
                
                # no more nodes to search from
                if len(node_list) == 0:
                    return []

        path = [dest.coord]
        parent = dest.parent
        while parent != None:
            path.append(parent.coord)
            parent = parent.parent
        
        path.reverse()
        return path
        
if __name__ == "__main__":
    global grid, stopevent
    grid = CozGrid("emptygrid.json")
    h = planning.heuristic
    tree = SearchTree(grid, h)
    tree.find_goal()

