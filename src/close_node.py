from scipy.spatial import KDTree

###DISCLAIMER
#I know that this has a problem with not recognizing angles that wrap around
#But for this purpose it is fine
#Plus I can't figure out how to handle this without looping thorugh all the points which would ruin the whole point of using a kd-tree

def is_point_within_distance(point, closed_list, distance):
    """
    Check if a point is within a certain distance of any point in the dictionary.

    Parameters:
    - point: tuple (x, y) representing the query point.
    - points_dict: dictionary {key: (x, y)} representing other points.
    - distance: float, the maximum distance to check.

    Returns:
    - bool: True if the point is within the distance of any dictionary point, False otherwise.
    """
    # Extract the points from the dictionary
    # points = list(closed_list)

    # Build a KDTree
    # print(closed_list)
    tree = KDTree(closed_list)

    # Query the KDTree
    indices = tree.query_ball_point(point, distance)

    return len(indices) > 0
