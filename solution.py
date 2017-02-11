import numpy as np
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, sokoban_goal_state #for Sokoban specific classes and problems
from test_problems import PROBLEMS

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan_distance = 0
    current_state = []
    goal_state = []
    for box in state.boxes:
      current_state.append(box)
    for storage in state.storage:
      goal_state.append(storage)
    for i in range(len(goal_state)):
      manhattan_distance+= abs(current_state[i][0]-goal_state[i][0])
      manhattan_distance+= abs(current_state[i][1]-goal_state[i][1])
    return manhattan_distance

def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    cost = 0
    if check_corners(state): return float("inf")
    cost += robot_beside_nothing(state)
    cost += distance(state)
    return cost 

def distance(state):
  final_cost = 0
  robot_distance = float("inf")
  robot_position = state.robot
  for box in state.boxes:
    possible_storage = get_possible_storage(box, state)
    tempcost = []
    old_cost = float("inf")
    for possible in possible_storage:
      if box == possible:
        old_cost = 0
        break
      else:
        new_cost = calculate_simple_distance(box, possible, state)
        if new_cost <= old_cost:
          old_cost = new_cost
    final_cost +=old_cost
    if box not in possible_storage:
      final_cost += get_closeness(box,state)
      new_robot_distance = calculate_simple_distance(robot_position, box, state)
      if new_robot_distance<robot_distance:
        robot_distance = new_robot_distance
  if robot_distance != float("inf"):
    final_cost += robot_distance
  return final_cost 

def get_closeness(box, state):
  cost = 0
  top = get_top(box)
  if top in state.obstacles: cost+=1
  if out_bound(top,state): cost+=1
  bottom = get_bottom(box)
  if bottom in state.obstacles: cost+=1
  if out_bound(bottom,state): cost+=1
  left = get_left(box)
  if left in state.obstacles: cost+=1
  if out_bound(left,state): cost+=1
  right = get_right(box)
  if right in state.obstacles: cost+=1
  if out_bound(right,state): cost+=1
  return cost

def out_bound(box, state):
  if box[0] <0: return True
  if box[1] <0: return True
  if box[0] >=state.width: return True
  if box[1] >=state.height: return True
  return False

def get_top(box):
  return (box[0],box[1]+1)
def get_bottom(box):
  return (box[0],box[1]-1)
def get_left(box):
  return (box[0]-1,box[1])
def get_right(box):
  return (box[0]+1,box[1])

def robot_beside_nothing(state):
  robot_position = state.robot
  cost = 0
  if (robot_position[0]+1, robot_position[1])  in state.boxes:
    test = (robot_position[0]+2, robot_position[1]) in state.boxes
    if test in state.boxes or test in state.obstacles:
      cost+= 2
    else:
      return cost
  if (robot_position[0]-1, robot_position[1])  in state.boxes:
    test = (robot_position[0]-2, robot_position[1]) in state.boxes
    if test in state.boxes or test in state.obstacles:
      cost+= 2
    else:
      return cost
  if (robot_position[0], robot_position[1]+1)  in state.boxes:
    test = (robot_position[0], robot_position[1]+2) in state.boxes
    if test in state.boxes or test in state.obstacles:
      cost+= 2
    else:
      return cost
  if (robot_position[0], robot_position[1]-1)  in state.boxes:
    test = (robot_position[0], robot_position[1]-2) in state.boxes
    if test in state.boxes or test in state.obstacles:
      cost+= 2
    else:
      return cost
  cost+=1
  if (robot_position[0]+1, robot_position[1]+1)  in state.boxes: return cost
  if (robot_position[0]-1, robot_position[1]-1)  in state.boxes: return cost
  if (robot_position[0]-1, robot_position[1]+1)  in state.boxes: return cost
  if (robot_position[0]+1, robot_position[1]-1)  in state.boxes: return cost
  return cost+2

def calculate_simple_distance(box, possible,state):
  return abs(box[0]-possible[0])+ abs(box[1]-possible[1])

def is_cornered(position, state):
  if position[0] == 0:
    if position[1] == 0: return True
    if position[1] == state.height-1: return True
    if (position[0], position[1]-1) in state.obstacles: return True
    if (position[0], position[1]+1) in state.obstacles: return True
    return False   
  if position[0] == state.width-1:
    if position[1] == 0: return True
    if position[1] == state.height-1: return True
    if (position[0]-1, position[1]) in state.obstacles: return True
    if (position[0]+1, position[1]) in state.obstacles: return True  
    return False  
  testabove = (position[0]-1, position[1])
  testbelow = (position[0]+1, position[1])
  testleft = (position[0], position[1]-1)
  testright = (position[0], position[1]+1)
  if testabove in state.obstacles:
    if testleft in state.obstacles: return True
    if testright in state.obstacles: return True
  if testbelow in state.obstacles:
    if testleft in state.obstacles: return True
    if testright in state.obstacles: return True
  return False

def check_corners(state):
  for box in state.boxes:
    possible_storage = get_possible_storage(box, state)
    if box not in possible_storage:
      if is_cornered(box, state): return True
      # if is_edge(box, possible_storage,state): return True
  return False

def get_possible_storage(box,state):
  if state.restrictions != None:
    possible = state.restrictions[state.boxes[box]]
    if box in possible:
      return [box]
    for other_boxes in state.boxes:
      if box != other_boxes:
        if other_boxes in possible and other_boxes in state.restrictions[state.boxes[other_boxes]]:
          possible = possible.difference(other_boxes)
    return possible
  else:
    possible = []
    for place in state.storage:
      possible.append(place)
    if box in possible:
      return [box]
    for other_boxes in state.boxes:    
      if box != other_boxes:
        if other_boxes in possible:
          possible.remove(other_boxes)  
    return possible


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    fval = sN.gval + weight*sN.hval
    return fval


def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    time_end = os.times()[0]+timebound
    search = SearchEngine('best_first')
    search.init_search(initial_state, sokoban_goal_state, heur_fn)
    time_left = time_end - os.times()[0]
    output = False
    prev_cost = float("inf")
    while time_left > 0:
      goal = search.search(time_left)
      if goal != False:
        if goal.gval<prev_cost:
          output = goal
          prev_cost = goal.gval
      else:
        break
      time_left = time_end - os.times()[0]
    return output

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    time_end = os.times()[0]+timebound
    search = SearchEngine('custom')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    time_left = time_end - os.times()[0]
    output = False
    prev_cost = float("inf")
    while time_left > 0:
      goal = search.search(time_left)
      if goal != False:
        if goal.gval<prev_cost:
          output = goal
          prev_cost = goal.gval
      else:
        break
      time_left = time_end - os.times()[0]
    return output
    return False

