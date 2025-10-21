import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
import numpy as np

ROBOTS = 3
GRIDS = OccupancyGrid()
GRIDS_FOR_AVERAGE = None
EXPLORED = 0.0
AREA_EXPLORED_PUB = None
AVG_AREA_EXPLORED_PUB = None

def compute_explored_cells(grid):
    """
    Compute the explored area from the occupancy grid.
    Counts cells that are known (not -1) excluding obstacles (100).
    In ROS occupancy grids:
    - -1: unknown
    - 0: free space
    - 1-99: low to high probability of obstacle
    - 100: definitely occupied
    """
    if not grid.data:
        rospy.logwarn("Grid data is empty")
        return 0
    
    explored_cells = 0
    for value in grid.data:
        # Count as explored if it's known (not -1) and not a definite obstacle (not 100)
        if value != -1 and value < 100:
            explored_cells += 1
    
    total_cells = len(grid.data)
    unknown_cells = sum(1 for value in grid.data if value == -1)
    obstacle_cells = sum(1 for value in grid.data if value == 100)
    
    rospy.logdebug(f"Grid analysis: Total={total_cells}, Explored={explored_cells}, "
                   f"Unknown={unknown_cells}, Obstacles={obstacle_cells}")
    
    return explored_cells

def cells_to_meters_squared(cells, resolution):
    """
    Convert the number of cells to meters squared based on the grid resolution.
    """
    if resolution <= 0:
        rospy.logerr(f"Invalid grid resolution: {resolution}")
        return 0.0
    
    area_m2 = cells * (resolution ** 2)
    rospy.logdebug(f"Converted {cells} cells to {area_m2:.2f} m² (resolution: {resolution})")
    return area_m2

def grid_callback(data):
    global GRIDS, EXPLORED
    GRIDS = data
    
    if not data.data:
        rospy.logwarn("Received empty grid data")
        return
    
    explored_cells = compute_explored_cells(data)
    explored_area = cells_to_meters_squared(explored_cells, data.info.resolution)
    EXPLORED = explored_area

def grid_lambda_callback(data, robot_id):
    global GRIDS_FOR_AVERAGE, EXPLORED
    GRIDS_FOR_AVERAGE[robot_id] = data

    if not data.data:
        rospy.logwarn("Received empty grid data")
        return

def update(event):
    global AREA_EXPLORED_PUB
    """Publish the average area explored by all robots.
    """
    if AREA_EXPLORED_PUB is None:
        rospy.logerr("AREA_EXPLORED_PUB is not initialized.")
        return
    
    msg = Float32()
    msg.data = EXPLORED
    AREA_EXPLORED_PUB.publish(msg)

    avg_msg = Float32()
    avg = []
    for i in range(ROBOTS):
        if not GRIDS_FOR_AVERAGE[i].data:
            rospy.logwarn(f"Grid data for robot {i} is empty")
            continue
        cells = compute_explored_cells(GRIDS_FOR_AVERAGE[i])
        area = cells_to_meters_squared(cells, GRIDS_FOR_AVERAGE[i].info.resolution)
        avg.append(area)    
        
    avg_msg.data = np.mean(avg)
    AVG_AREA_EXPLORED_PUB.publish(avg_msg)

    rospy.loginfo_throttle(5, f"(Does not represent the truth) Joint explored area: {EXPLORED:.2f} m² (Truth) Average: {avg_msg.data:.2f} m²")

def main():
    global ROBOTS, GRIDS, AREA_EXPLORED_PUB, AVG_AREA_EXPLORED_PUB, GRIDS_FOR_AVERAGE
    namespace = rospy.get_namespace()
    rospy.init_node('data_analysis', anonymous=True)
    rospy.Subscriber("/global_fusion_statistics", OccupancyGrid, grid_callback)
    ROBOTS = rospy.get_param('~robots', default=1)

    GRIDS_FOR_AVERAGE = [OccupancyGrid() for _ in range(ROBOTS)]

    for i in range(ROBOTS):
        grid_topic = "/robot_{}/fusion".format(i)
        rospy.Subscriber(grid_topic, OccupancyGrid, grid_lambda_callback, callback_args=i)

    AREA_EXPLORED_PUB = rospy.Publisher('/area_explored', Float32, queue_size=10)
    AVG_AREA_EXPLORED_PUB = rospy.Publisher('/avg_area_explored', Float32, queue_size=10)
    timer = rospy.Timer(rospy.Duration(1.0), update)

    rospy.loginfo(f"Data analysis node started for {ROBOTS} robots")    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down data analysis node")
    finally:
        timer.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass