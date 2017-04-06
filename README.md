# Sokoban-Puzzle-Solver
Assignment for Artificial Intelligenc CSC384

Note that the sokoban environment was provided for the course. The heuristics that I implemented are in solutions.py

Implemented a cost heuristic to prune/optimize tree search to solve Sokoban Puzzle within time contraint.

A few ways that cost heuristic operated:
1.In order to limit the search space, “dead lock” positions such as when a box is stuck in a corner and not in a storage position, should always be avoided. This is because when a box is stuck in a corner, there is no way for the robot to move the box as the robot can only push boxes.
2.When boxes are beside either an obstacle or other box, its degrees of freedom are limited (i.e. if an obstacle is directly to the left of a box, the box can no longer move away from the obstacle to the right or move to the left). Thus states with boxes having limited degrees of freedom should be minimized.
3.The minimum distance of the robot to a box should be minimized. This is to ensure that the robot is near a box so that it is more likely to move boxes around and solve the problem.
4.Because each problem is to be solved in a set amount of time, code efficient is quite important. Thus it is better to determine states with “dead lock” first such they can be immediately be eliminated without needing to calculate a heuristic cost for that state. 
5.When calculating the distance between a box and storage positions, be sure that when calculating a distance cost, to use the closest storage position available to that box. This is because it is more realistic that a box’s target storage position is the closest one and not a storage position farther away. However be sure that not all the boxes are using the same storage position when calculating costs because only a single box can eventually occupy that position.
