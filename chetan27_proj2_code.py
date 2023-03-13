import numpy as np
import heapq as hq
from matplotlib import pyplot as plt



#Actions and their costs 
Action_set = [ ((0,1),1), ((0,-1),1), ((1,0),1), ((-1,0),1),
               ((1,1),1.4),((-1,1),1.4), ((1,-1),1.4), ((-1,-1),1.4) ]
          
class Node:
    def __init__(self,x,y,c2c,parent_co):
        self.x = x
        self.y = y
        self.c2c = c2c
        self.parent_co = parent_co
        self.co = (x,y)
    def __lt__(self,other):
        return self.c2c < other.c2c


def plot(path_list, closed_list, initial_node, goal_node):
     
     # plotting 
     print("Plotting...")
     
     x_path = []
     y_path = []
     e_x =[]
     e_y =[] 

     for co_or in path_list:
       x_path.append(co_or[0])
       y_path.append(co_or[1])
     
     for ele in closed_list:
       e_x.append(ele[0])
       e_y.append(ele[1]) 

     # plotting explored nodes
     plt.plot(e_x, e_y, 'g*')
    
     #plotting path 
     plt.plot(x_path, y_path, "b-")

     # plotting initial nodes and goal nodes 
     plt.plot(initial_node.x, initial_node.y, 'rp')
     plt.plot(goal_node.x, goal_node.y, 'rp')
   
     #plot map
     plt.imshow(map,"gray")
     ax = plt.gca()
     ax.invert_yaxis()
     plt.show()



#inputs
p = int(input("enter the initial node x co-ordinate "))
q = int(input("enter the initial node y co-ordinate "))
r = int(input("enter the goal node x co-ordinate "))
s = int(input("enter the goal node y co-ordinate "))

if( p<0 or p>601 or q<0 or q>251 or r<0 or r>601 or s<0 or s>251 ):
    print("node(s) co-ordinates are out of bounds")
    quit()

#defining lists and dictionaries
initial_node = Node(p,q,0,None)
goal_node = Node(r,s,np.inf,None)
path_list = []
open_list =[]    #co-ordinate-->c2c
closed_set = set()
closed_list = []
info_dict = {}   #child-->parent
cost_dict = {}

hq.heappush(open_list, [initial_node.c2c, initial_node])

#map with obstacles 
map = np.zeros([251,601], dtype =int)
for y in range(251):
    for x in range(601):
        #Box obstale
        b1 = (x) - 150
        b2 = (x) - 100
        b3 = (y) - 100
        b4 = (y) - 150
        


        #triangle obstacle
        t1 = (x) - 460
        t2 = (y) + 2 * (x) - 1145
        t3 = (y) - 2 * (x) + 895
 

        #hexagon obstacle
        h1 = y - 0.577 * x + 123.21
        h2 = x - 364.95
        h3 = y + 0.577 * x - 373.21
        h4 = y - 0.577 * x - 26.92
        h5 = x - 235
        h6 = y + 0.577 * x - 223.08


        if( (t1 > 0 and t2 < 0 and t3 > 0) or (b1<0 and b2>0 and b3 <0 ) or (b1<0 and b2>0 and b4 > 0)  or (h2 < 0 and  h5 > 0 and h1 > 0 and h3 < 0 and h4 < 0 and h6 > 0) ):
            map[y,x] = 1

#exits if start node or goal node is in obstacle space 
if(map[initial_node.y,initial_node.x] == 1  or map[goal_node.y,goal_node.x]==1):
    print("Failed!!! start node or goal node is in obstacle space")
    quit()


# Dijkstra Algorithm
while( open_list != [] ) :
    
    current_node = (hq.heappop(open_list))[1]

    if(current_node.co in closed_set): continue 

    else: 
        closed_set.add(current_node.co)
        closed_list.append(current_node.co)
        info_dict[current_node.co] = current_node.parent_co

        if(current_node.co == goal_node.co):
             print("Found Goal Node!!!")
             #back_tracking 
             path_list.append(current_node.co)
             parent_dash = info_dict.get(current_node.co)

             while parent_dash != None:  

              path_list.append(parent_dash)
              parent_dash = info_dict.get(parent_dash)

             path_list.reverse()
             # calling plot function 
             plot(path_list, closed_list, initial_node, goal_node)
             break
        

        
        #searching and creating new nodes 
        for element in Action_set:

            x = current_node.x + element[0][0]
            y = current_node.y + element[0][1]
            cost = element[1]

            if(  (x in range(0,601))  and (y in range(0,251)) and (map[y,x] != 1) and  ((x,y) not in closed_set)  ):

                if( (x,y) not in cost_dict.keys() ):
                    k = cost + current_node.c2c
                    x_dash_node = Node(x,y,k,current_node.co)
                    cost_dict[(x,y)] = x_dash_node
                    hq.heappush(open_list,[k,x_dash_node])
                
                else:
                    x_dash_node = cost_dict.get((x,y))
                    if(x_dash_node.c2c > cost + current_node.c2c):
                        x_dash_node.c2c = cost + current_node.c2c
                        x_dash_node.parent_co = current_node.co
                        cost_dict[(x,y)] = x_dash_node
                        





    






    



