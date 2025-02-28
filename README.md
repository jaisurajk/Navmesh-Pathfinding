## Objective

I implemented the A* search algorithm and the bidirectional A* algorithm to find the shortest path between two points drawn on an image. The image the path is located on is called a navmesh because the image itself is a meshgrid where I can plot multiple points. In addition, I have to navigate the meshgrid to compute two points on the grid that make up the shortest path.  

To do this, I implemented 4 functions: 
- find_points(): Calculates the point where a path enters the next bounding grid-space of a meshgrid.
- find_distance(): Calculates the Euclidean distance between a given point and the entry point.
- heuristic_cost_estimate: Calculates the Euclidean distance between a given point and a goal point based on the weight of each segment of the path that is formed by these two points.
- find_path(): Finds the shortest path between two points within a navmesh using bidirectional A* search algorithm. The points of the navigated paths are stored using heappush().
  
Goal of mcts_vanilla.py: Beat Rollout_bot most of the time for a tree size of 1000 nodes.

Goal of mcts_modified.py: Beat Rollout_bot most of the time for a tree size of 1000 nodes using my unique heuristic approach.

## MODIFICATION OF MCTS_MODIFIED.PY

In my modifications to 'mcts_modified.py', I focused on enhancing the performance by adding a heuristic approach during the rollout phase. I introduced the 'heuristic_choice' function, which uses the 'next_move' function to score possible moves. The 'heuristic_choice' function iterates through all potential moves, evaluates each one, and selects the move with the highest score. The 'next_move' function assigns scores based on specific criteria: it gives a high score of 10 if the move results in a win for the bot, a score of 5 if the move blocks the opponent's win, and a score of 1 for any other move. This heuristic helps the program by making the bot prioritize moves that lead to immediate wins or prevent the opponent from winning, thus making the bot smarter and more effective.
