import math
from heapq import heappop, heappush

def find_points(px, py, current, next): 
    """
    Determines detail points (exact points where path enters next box)
    """
    new_x, new_y = 0, 0
    b1x1, b1x2, b1y1, b1y2 = current
    b2x1, b2x2, b2y1, b2y2 = next

    left, right = max(b1x1, b2x1), min(b1x2, b2x2)
    bottom, top = max(b1y1, b2y1), min(b1y2, b2y2)

    if right - left != 0:
        if px < left: new_x = left
        elif px > right: new_x = right
        else: new_x = px
        new_y = bottom

    if top - bottom != 0:
        if py < bottom: new_y = bottom
        elif py > top: new_y = top
        else: new_y = py
        new_x = left

    elif right - left == 0 and top - bottom == 0:
        if px < left: new_x = left
        elif px > right: new_x = right
        else: new_x = px
        if py < bottom: new_y = bottom
        elif py > top: new_y = top
        else: new_y = py

    return new_x, new_y

def find_distance(coords, current, next):
    """
    finds euclidean distance between two boxes detail points
    """
    x1, y1 = coords
    x2, y2 = find_points(x1, y1, current, next)
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist, (x2, y2)

def heuristic_cost_estimate(point, goal):
    return ((point[0] - goal[0])**2 + (point[1] - goal[1])**2)**0.5

def find_path(source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh
    """
    path = [destination_point]
    detail_points = {}
    forward_distances = {}
    backward_distances = {}
    boxes = {}

    src_x, src_y = source_point
    dst_x, dst_y = destination_point
    src_box = tuple()
    dst_box = tuple()

    for key, value in mesh.items():
        if key == "boxes":
            for i in value:
                x1, x2, y1, y2 = i
                if (src_x > x1 and src_x < x2) and (src_y > y1 and src_y < y2): 
                    src_box = i
                    boxes[src_box] = None
                if (dst_x > x1 and dst_x < x2) and (dst_y > y1 and dst_y < y2):
                    dst_box = i
                    boxes[dst_box] = None
                if src_box and dst_box:
                    break
    
    #print(f"Source Point: {source_point}, Source Box: {src_box}")
    #print(f"Destination Point: {destination_point}, Destination Box: {dst_box}")

    if src_box == dst_box:
        return [source_point, destination_point], boxes.keys()

    if src_box is None or dst_box is None:
        print("Source or destination point is outside the bounds of the boxes.")
        return [], boxes.keys()

    forward_queue = []
    backward_queue = []
    heappush(forward_queue, (0, src_box))
    heappush(backward_queue, (0, dst_box))

    forward_prev = {}
    backward_prev = {}
    detail_points[src_box] = source_point
    detail_points[dst_box] = destination_point
    forward_distances[src_box] = 0
    backward_distances[dst_box] = 0

    while forward_queue and backward_queue:
        # Process the forward search
        forward_priority, forward_current = heappop(forward_queue)
        for next in mesh["adj"][forward_current]:
            cost, points = find_distance(detail_points[forward_current], forward_current, next)
            cost_to_adj = forward_distances[forward_current] + cost
            if next not in forward_distances or cost_to_adj < forward_distances[next]:
                detail_points[next] = points
                forward_distances[next] = cost_to_adj
                p = cost_to_adj + heuristic_cost_estimate(points, destination_point)
                heappush(forward_queue, (p, next))
                forward_prev[next] = forward_current

        # Process the backward search
        backward_priority, backward_current = heappop(backward_queue)
        for next in mesh["adj"][backward_current]:
            cost, points = find_distance(detail_points[backward_current], backward_current, next)
            cost_to_adj = backward_distances[backward_current] + cost
            if next not in backward_distances or cost_to_adj < backward_distances[next]:
                detail_points[next] = points
                backward_distances[next] = cost_to_adj
                p = cost_to_adj + heuristic_cost_estimate(points, source_point)
                heappush(backward_queue, (p, next))
                backward_prev[next] = backward_current

        # Check for intersection between forward and backward searches
        if forward_current in backward_prev or backward_current in forward_prev:
            meeting_point = forward_current if forward_current in backward_prev else backward_current
            break
    else:
        print("No Path")
        return [], boxes.keys()

    # Build path
    forward_path = []
    current = meeting_point
    while current != src_box:
        forward_path.append(detail_points[current])
        boxes[current] = None
        current = forward_prev[current]
    forward_path.append(detail_points[src_box])
    forward_path.append(source_point)
    forward_path.reverse()

    backward_path = []
    current = meeting_point
    #print(detail_points[dst_box])
    while current != dst_box:
        backward_path.append(detail_points[current])
        boxes[current] = None
        current = backward_prev[current]
    backward_path.append(detail_points[dst_box])
    backward_path.append(destination_point)

    path = forward_path + backward_path[1:]  # Merge paths
    return path, boxes.keys()



