import numpy as np

from line_circle import intersect_line_circle

# Calculate the virtual force exerted by the original trajectory on the "follower."
def calculate_mov_direction(b3,b2,b1,obj_point,f1,f2,f3,kat1,kat2,kat3,kre1,kre2,kre3):
    # Magnitude of attraction to the next point
    att1 = (np.array(f1) - np.array(obj_point))
    # Magnitude of attraction to the next two points
    att2 = (np.array(f2) - np.array(obj_point))
    # Magnitude of attraction to the next three points
    att3 = (np.array(f3) - np.array(obj_point))
    # Magnitude of repulsion from the previous point
    req1 = (np.array(obj_point) - np.array(b1))
    # Magnitude of repulsion from the previous two points
    req2 = (np.array(obj_point) - np.array(b2))
    # Magnitude of repulsion from the previous three points
    req3 = (np.array(obj_point) - np.array(b3))
    ## Combine attraction and repulsion
    return kat1 * np.array(att1) + kre1 * np.array(req1) + kat2 * np.array(att2) + kre2 * np.array(req2) + kat3 * np.array(att3) + kre3 * np.array(req3)

def calculate_mov_direction_total(b3, b2, b1, obj_point, f1, f2, f3, kat1, kat2, kat3, kre1, kre2, kre3,obstacles,kre):
    #没有障碍物时的合力
    force_origin = np.array(calculate_mov_direction(b3,b2,b1,obj_point,f1,f2,f3,kat1,kat2,kat3,kre1,kre2,kre3))

    end_point = f1
    req_total = np.array((0,0))
    for obstacle in obstacles:
        x_obstacle = obstacle['x']  # Get x-coordinate of the obstacle's center
        y_obstacle = obstacle['y']  # Get y-coordinate of the obstacle's center
        r_obstacle = obstacle['r']  # Get radius of the obstacle
        p_obstacle = obstacle['p']  # Get influence range of the obstacle
        obstacle_point = (x_obstacle,y_obstacle)
        intersection = intersect_line_circle(obstacle_point, r_obstacle, obj_point, end_point)
        if intersection is not None:

            dis = np.linalg.norm(np.array(obj_point) - np.array(intersection))


            if dis < p_obstacle:
                # Magnitude of attraction.
                att_dis = np.linalg.norm(np.array(end_point) - np.array(obj_point))  # 吸引力大小
                # Magnitude of repulsion.
                req_dis = np.linalg.norm(np.array(obj_point) - np.array(obstacle_point))
                max = att_dis * 1.1
                # The maximum value of repulsion is how many times the attraction, to prevent sudden changes.
                req_p = - max / p_obstacle * dis + max
                # The appropriate magnitude of repulsion.
                req = req_p/req_dis * (np.array(obj_point) - np.array(obstacle_point))
                # The final repulsive force.
            else:
                req = (0,0)
                # The final repulsive force.
        else:
            req = (0, 0)
            # The final repulsive force.
        req_total = req_total + np.array(req)

    return force_origin + kre * np.array(req_total)
    # Combine attraction and repulsion.

