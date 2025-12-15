# Watering Problem
The _general_ problem that we wish to solve is to find the _optimal_ way to water a garden full of plants.
To do so, we employ a variety of AI strategies, such as path finding ($A^*$, GBFS), Markov Decision Processes, and more!

## What even is optimality?
This is a very good question. Sometimes we'd want to find the shortest sequence of actions that would assure all plants are satiated, and sometimes, we just wish to find the best action at any given moment.
When we decide what we consider as being "optimal", then we can start considering what would be the appropriate AI-strategy to solve it.

## Shortest Path Finding
When we talk about finding the short-*est* path, or the big-*est* something, we have no choice but to go over all possibles values of that thing.
Intuitively, this means that if we just go over all the possible paths, rule out those which don't end up in a solution, and then choose out of the remaining paths the shortest one -- we'd end up with the shortest path of all.
#### Branching branching branching
This _naïve_ approach really would work great for small problems, BUT, when there are MILLIONS and MILLIONS of possible paths, considering each and every single one of them would take ages!
For this reason, the primary way we should strive to optimise a planning algorithm, is by ruling out or "pruning" bad paths as early as possible. If we think aboout it mathematically, in each state of this problem, we had several ($R$), where each robot could move UP, DOWN, LEFT and RIGHT, and it could also LOAD / POUR water, giving us a total of $~6$ possible actions per robot. This means that for every path that did not yet reach the goal state (where all plants are satiated) we have to admit $6R$ new states.
This numebr, is called the _branching factor_ of the algorithm, and to repeat what we said formally, the primary way we should strive to optimise a planning algorithm, is by minimising the branching factor.

### Bitter Experience Optimising Redundantly
Being the first planner I ever wrote, I struggled to believe that it really is the case that only the branching factor matters. For this reason, I tried optimising my actual CODE, by refactoring to use different datastructures, optimising state-hashing, and more.
All these brought mild, improvements, cumulatively changing the runtime by roughly $1%$.

### Heuristics
A heuristic is a function that, given a state, estimates the number of remaining steps we have to take in order to arrive at the goal state, we use this number to estimate what is the state we should develop at any particular time.
The major heuristic improvements divide into two sections, those that improved its runtime, and those which improved its estimation:
To improve the heuristic's accuracy, one needs to think about what really are the steps we know for a FACT we need to perform in order to complete the goal. For example, if we need to water a garden and we know that we have no robot with loaded water, then we have to visit a tap, then a plant no matter what. For this reason, we can add the minimal amount of actions required to reach a tap, load water from it then reach a plant and pour water onto it to our heuristic. Additionally, heuristics tend to work with distances, and this brings up the question of how do we measure distance on a grid where there are obstacles (walls and other robots) blocking the path. Our approach, used BFS to pre-compute all actual distances between two points on the grid which drastically improved the heuristic in cases where manhatten distances was completely off because of obstacles (for example if we have a grid that is the shape of a snake, with an entire line of obstacles except for a single cell that is free, the robot needs to walk an entire line, pass the obstacle and then walk an entire line back just to simulate going two steps upwards.
To improve the heuristic's runtime, the best move is to simply pre-calculating a lot of constant information such as distances between different locations in the garden.

### Caching
To employ this method, we have to use a heuristic that is _consistent_, meaning intuitively that if we took $k$ steps, the heuristic could not have increased by more than $k$.
If such is the case, then the $A^*$ algorithm develops every state with the shortest path to it first, meaning if we have a state $s$, we could not arrive at it in $10$ steps and only THEN find a path leading to it in $9$ steps.
For this reason, we can cache the states that we visited, and before returning a state by the _successor_ function, check whether it is already in the cache, and if so, prune it. This approach reduced my runtime by roughly $25%$.

### Pre-calculating
Always pre-calculate anything that does not depend on a specific state! After having pre-computed many things, you suddenly have loads of tools at your disposals, that you can use for advanced calculations and tight heuristics during the runtime at no further cost.
