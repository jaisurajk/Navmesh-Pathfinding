## Objective

I implemented the A* search algorithm and the bidirectional A* algorithm to find the shortest path between two points drawn on an image. The image the path is located on is called a navmesh because the image itself is a meshgrid where I can plot multiple points. In addition, I have to navigate the meshgrid to compute two points on the grid that make up the shortest path.  

nm_pathfinder.py:

To do this, I implemented 4 functions: 
- find_points(): Calculates the point where a path enters the next bounding grid-space of a meshgrid.
- find_distance(): Calculates the Euclidean distance between a given point and the entry point.
- heuristic_cost_estimate(): Calculates the Euclidean distance between a given point and a goal point based on the weight of each segment of the path that is formed by these two points.
- find_path(): Finds the shortest path between two points within a navmesh using bidirectional A* search algorithm. The points of the navigated paths are stored using heappush().
  
## Files

- nm_interactive.py: Main program that takes in 3 arguments: a PNG image, the filename of a .mesh.pickle, and a scaler.
- nm_meshbuilder.py: Builds navmeshes for the inputted PNG image.

## Running

The command used to run the program is "python nm_interactive.py ../input/PNGimage ../input/PNGimage.mesh.pickle scaler".

Here's an example PNG file: 
<img width="184" alt="Screenshot 2025-03-03 at 1 24 11 PM" src="https://github.com/user-attachments/assets/0fd7e8fc-dc58-4ce0-a1cb-a020280d8d03" />

Here's the output:
<img width="219" alt="Screenshot 2025-03-03 at 1 24 55 PM" src="https://github.com/user-attachments/assets/242ef9d3-181e-4635-8880-c26671823540" />

