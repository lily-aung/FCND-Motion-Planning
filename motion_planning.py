import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
from numpy.lib.function_base import _calculate_shapes

from planning_utils import a_star, heuristic, create_grid,prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection,global_goal_position=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []

        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        print('setting global_goal_position')
        print(global_goal_position)
        self.global_goal_position = global_goal_position
        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.01:
                if abs(self.local_position[2]) < 0.1:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        with open(filename) as f:
            tmp = f.readline()
        lat0,lon0=np.float64(tmp.split(',')[0].replace('lat0','')), np.float64(tmp.split(',')[1].replace('lon0',''))
        print('lat0:{},lon0:{}'.format(lat0,lon0))
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0,lat0, 0)
        # TODO: retrieve current global position
        global_position_lon,global_position_lat,global_position_alt = self.global_position[0],self.global_position[1],self.global_position[2]
        # TODO: convert to current local position using global_to_local()
        start_loc = global_to_local(self.global_position,self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,self.local_position))
        # Read in obstacle map
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset= create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.ceil(start_loc[0] - north_offset)) , int(np.ceil(start_loc[1] - east_offset)))
        print ("grid_Start :",grid_start)
        # Set goal as some arbitrary position on the grid
        #test_pos1 = (-122.397745, 37.793837)
        #goal1 = [ *test_pos1, 0]
	    #goal1 = [*self.global_goal_position,0]
        #goal1 = [*self.global_goal_position]
        #print('goal1 :',goal1)
        # TODO: adapt to set goal as latitude / longitude position and convert
        #grid_goal = (int(np.ceil(goal_loc[0] - north_offset)),int(np.ceil(goal_loc[1] - east_offset)))
        goal_loc = global_to_local(self.global_goal_position, self.global_home)
        grid_goal = (int(np.ceil(goal_loc[0] - north_offset)), int(np.ceil(goal_loc[1] - east_offset)))

        print ("grid_goal :",grid_goal)
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        pruned_path = prune_path(path)
        print('Number of waypoints: {}'.format(len(path)))
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        #waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
	    #self.waypoints = calculate_waypoints(self.global_position, goal_global_position, self.global_home, data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        # Only required if they do threaded
        # while self.in_mission:
        #    pass
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--global_goal_lon', type=str, default='-122.397755', help="Goal position longitude")#-122.39774,-122.40195876,-122.40199327
    parser.add_argument('--global_goal_lat', type=str, default='37.793839', help="Goal position latitude")#37.793837,37.79673913,37.7902035 
    parser.add_argument('--global_goal_alt', type=str, default='0', help="Goal position altitude")

    args = parser.parse_args()
    print(args._get_kwargs())

    def find_arg_val(key=None, default_value=20.0):
        v = [tuple[1] for tuple in args._get_kwargs() if tuple[0] == key][0]
        if v is None:
            return float(default_value)
        return float(v)
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    global_goal_position  = ( find_arg_val(key='global_goal_lon')  , find_arg_val(key='global_goal_lat') , find_arg_val(key='global_goal_alt') )
    print(' ---- global_goal_position ----')
    print(global_goal_position)
    drone = MotionPlanning(conn, global_goal_position=global_goal_position)
    #drone = MotionPlanning(conn)
    time.sleep(1)
    drone.start()
