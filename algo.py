import pygame
import math
from queue import PriorityQueue, Queue

#Colour RGB Hex codes
HEX_AREA = (143, 221, 223) #Light Blue
HEX_BORDER = (254, 254, 254)#Almost White (right now) to account for the circle borders
HEX_BLUE = (0, 0, 255)
HEX_WHITE = (255, 255, 255)
HEX_WALLS = (30, 63, 102) #Black/Blue
HEX_PATH = (255, 255, 0) #Yellow
HEX_GREY = (128, 128, 128)
HEX_START = (172, 223, 135) #Navy blue
HEX_END = (250, 107, 132) 

#Instantiating the window GUI using pygame
WIDTH = 800
WINDOW = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Pathfinding Visualization")

#Class for each grid spot representative of a node in an unweighted undirected graph 
#   (distance between each square can be 1)
#Values to remember: 
#   position (x,y),
#   own width (for GUI), 
#   keep track of neighboring nodes, 
#   colour which represents the property (active/inactive/start/end)
class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.colour = HEX_WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
    
    def get_pos(self):
        return self.row, self.col

    #Node states, referencing to colour
    #Determining whether a node has a property
    def is_closed(self):
        return self.colour == HEX_AREA
    def is_open(self):
        return self.colour == HEX_BORDER
    def is_wall(self):
        return self.colour == HEX_WALLS
    def is_start(self):
        return self.colour == HEX_START
    def is_end(self):
        return self.colour == HEX_END

    #Setting the node to a property
    def reset(self):
        self.colour = HEX_WHITE
    def make_closed(self):
        self.colour = HEX_AREA
    def make_open(self):
        self.colour = HEX_BORDER
    def make_wall(self):
        self.colour = HEX_WALLS
    def make_start(self):
        self.colour = HEX_START
    def make_end(self):
        self.colour = HEX_END
    def make_path(self):
        self.colour = HEX_PATH

    def draw(self, window):
        pygame.draw.rect(window, self.colour, (self.x, self.y, self.width, self.width))
        if self.colour == HEX_BORDER:
            pygame.draw.circle(window, (169, 238, 209), (self.x+self.width//2, self.y+self.width//2), self.width//2.4)

    #update the neighbors of a node
    def update_neighbors(self, grid):
        self.neighbors = []
        #Check DOWN neighbor 
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_wall():
            self.neighbors.append(grid[self.row + 1][self.col])
        #Check UP neighbor 
        if self.row > 0 and not grid[self.row - 1][self.col].is_wall():
            self.neighbors.append(grid[self.row - 1][self.col])
        #Check LEFT neighbor 
        if self.col > 0 and not grid[self.row][self.col - 1].is_wall():
            self.neighbors.append(grid[self.row][self.col - 1])
        #Check RIGHT neighbor 
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_wall():
            self.neighbors.append(grid[self.row][self.col + 1])


#H-function (Heuristic) used to estimate the distance from end node. Using the manhattan 
#   distance between nodes to calculate.
def h(a, b):
    x1, y1 = a
    x2, y2 = b
    return abs(x2 - x1) + abs(y2 - y1)

#A* algorithm using required parameters and the draw function called as a lambda function
def a_star_algorithm(draw, grid, start, end):
    #Keeping track of the node inserted first to break ties (equal f-scores in the search algorithm)
    count = 0

    #Using priority queue imported as an abstract data structure.
    open_set = PriorityQueue()
    #Pushing the start node into the open set
    open_set.put((0, count, start))

    #Keeping track of previous node in a dictionary to record the path, the reconstruct_path 
    #   function iterates backwards through the dictionary to draw the path
    came_from = {}
    
    #Dictionary for node g-scores recorded as infinity at the start. "List comprehension" 
    #   iterating throughout the grid of nodes.
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    #f-score
    f_score = {node: float("inf") for row in grid for node in row}
    #Using the heuristic between the start/end node to create an estimate of the shortest 
    #   path in order to avoid any gross miscalculations
    f_score[start] = h(start.get_pos(), end.get_pos())

    #Need to check if values are added/removes from the priorityQueue, 
    open_set_hash = {start}

    #Algorithm runs until the open set is empty, if it is empty: we have considered all 
    #   possible nodes (therefore if path have not been found, DNE)
    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        #Taking the third index to access the node from the open set. Popping the lowest
        #   value f-score from the open set (PriorityQueue)
        #Then simultaneously remove from open_set dictionary to keep track of the node just 
        #   popped from the priorityQueue in the dictionary
        current = open_set.get()[2]
        open_set_hash.remove(current) 
        
        #If reached the end node: create path
        if current == end:
            #Changing the border nodes to closed ones
            for row in grid:
                for node in row:
                    if node.colour == HEX_BORDER:
                        node.make_closed()

            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        #Check all of the neighbors, assuming all of the edges is 1, if we want to guess 
        #the g_score for each neighbor (provided we visit it next) just +1 to current g_score
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            #Checking for new best path and if so, updating f-score and g-score
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                #Adding the new node to the queue and in open_set_hash to keep track of 
                #   it being added to the queue
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()  

    return False

def breadth_first_algorithm(draw, start, end):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    
    while not frontier.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        current = frontier.get()
        #print(current.neighbors)
        
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        
        for neighbor in current.neighbors:
            if neighbor not in came_from:
                frontier.put(next)
                came_from[next] = current
                neighbor.make_open()
    
        draw()

        if current != start:
            current.make_closed()  

    return False


def dijsktras_algorithm(draw, grid, start, end):
    #Keeping track of the node inserted first to break ties (equal f-scores in the search algorithm)
    count = 0

    #Using priority queue imported as an abstract data structure.
    open_set = PriorityQueue()
    #Pushing the start node into the open set
    open_set.put((0, count, start))

    #Keeping track of previous node in a dictionary to record the path, the reconstruct_path 
    #   function iterates backwards through the dictionary to draw the path
    came_from = {}
    
    #Represents "cost-so-far"
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0


    #Need to check if values are added/removes from the priorityQueue, 
    open_set_hash = {start}

    #Algorithm runs until the open set is empty, if it is empty: we have considered all 
    #   possible nodes (therefore if path have not been found, DNE)
    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        #Taking the third index to access the node from the open set. Popping the lowest
        #   value f-score from the open set (PriorityQueue)
        #Then simultaneously remove from open_set dictionary to keep track of the node just 
        #   popped from the priorityQueue in the dictionary
        current = open_set.get()[2]
        open_set_hash.remove(current) 
        
        #If reached the end node: create path
        if current == end:
            #Changing the border nodes to closed ones
            for row in grid:
                for node in row:
                    if node.colour == HEX_BORDER:
                        node.make_closed()
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        #Check all of the neighbors, assuming all of the edges is 1, if we want to guess 
        #the g_score for each neighbor (provided we visit it next) just +1 to current g_score
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            #Checking for new best path and if so, updating f-score and g-score
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                #Adding the new node to the queue and in open_set_hash to keep track of 
                #   it being added to the queue
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((g_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()  

    return False


#Update the path on the UI using the found path, loop backwards in keys in the dictionary
def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


#Making a grid of nodes
def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)

    return grid

#Drawing out gridlines
def draw_grid(window, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(window, HEX_GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(window, HEX_GREY, (j * gap, 0), (j * gap, width))

#Drawing nodes using draw method in Node Class and drawing the gridlines
def draw(window, grid, rows, width):
    window.fill(HEX_WHITE)

    for row in grid:
        for node in row:
            node.draw(window)
    
    draw_grid(window, rows, width)
    pygame.display.update()

#Take the mouse position and extrapolate which node is being hovered on
def get_clicked_postion(pos, rows, width):
    gap = width // rows
    x, y = pos

    col = x // gap
    row = y // gap

    return col, row


##MAIN##
def main(window, width):
    ROWS = 40
    grid = make_grid(ROWS, width)

    start = None
    end = None

    #Checking some states
    run = True
    started = False

    #Modes for different search algorithms (0: breadth-first search, 1:dijsktra, 2: a*search)
    algorithm_picked = None

    print("""To Visualize a Path, select start, end nodes. 
Select From the Following:
    [1] Breadth-first Search 
    [2] Dijkstra's Algorithm
    [3] A-star Search

    and press [SPACE] to visualize.
    
Draw borders around the grid to truly see the algorithms at work!""")

    while run:
        draw(window, grid, ROWS, width)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            
            #If the algorithm is started, don't check for any other inputs by the user.
            if started:
                continue
            #Left Mouse
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_postion(pos, ROWS, width)
                node = grid[row][col]

                if not start and node != end:
                    start = node
                    start.make_start()
                elif not end and node != start:
                    end = node
                    end.make_end()

                elif node != start and node != end:
                    node.make_wall()

            #Right Mouse
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_postion(pos, ROWS, width)
                node = grid[row][col]
                node.reset()

                if node == start:
                    start = None
                elif node == end:
                    end = None

            #Coding the Algorithm
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end and not started:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)

                    if algorithm_picked == 1:
                        print("Visualizing Breadth-first Search Algorithm")
                        breadth_first_algorithm(lambda: draw(window, grid, ROWS, width), start, end)
                        algorithm_picked == None

                    elif algorithm_picked == 2:
                        print("Visualizing Dijsktra's Algorithm")
                        dijsktras_algorithm(lambda: draw(window, grid, ROWS, width), grid, start, end)
                        algorithm_picked == None

                    elif algorithm_picked == 3:
                        #Calling the a* algorithm using draw function (lambda) and grid and start/end positions
                        print("Visualizing A* Search Algorithm")
                        a_star_algorithm(lambda: draw(window, grid, ROWS, width), grid, start, end)
                        algorithm_picked == None

                if event.key == pygame.K_1:
                    algorithm_picked = 1
                    print("Selected Breadth-first Search Algorithm")
                if event.key == pygame.K_2:
                    algorithm_picked = 2
                    print("Selected Dijsktra's Algorithm")
                if event.key == pygame.K_3:
                    algorithm_picked = 3
                    print("Selected A* Search Algorithm")

                #Clear grid
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()

if __name__ == "__main__":
    main(WINDOW, WIDTH)
