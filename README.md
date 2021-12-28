# FCND - 3D Motion Planning
![Quad Image](./images/enroute.png)



This project is a continuation of the Backyard Flyer project where you executed a simple square shaped flight path. In this project you will integrate the techniques that you have learned throughout the last several lessons to plan a path through an urban environment. Check out the [project rubric](https://review.udacity.com/#!/rubrics/1534/view) for more detail on what constitutes a passing submission.

## Option to do this project in a GPU backed virtual machine in the Udacity classroom!
Rather than downloading the simulator and starter files you can simply complete this project in a virual workspace in the Udacity classroom! Follow [these instructions](https://classroom.udacity.com/nanodegrees/nd787/parts/5aa0a956-4418-4a41-846f-cb7ea63349b3/modules/0c12632a-b59a-41c1-9694-2b3508f47ce7/lessons/5f628104-5857-4a3f-93f0-d8a53fe6a8fd/concepts/ab09b378-f85f-49f4-8845-d59025dd8a8e?contentVersion=1.0.0&contentLocale=en-us) to proceed with the VM. 

## To complete this project on your local machine, follow these instructions:
### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```
### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project. 

### Step 5: Inspect the relevant files
For this project, you are provided with two scripts, `motion_planning.py` and `planning_utils.py`. Here you'll also find a file called `colliders.csv`, which contains the 2.5D map of the simulator environment. 

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py`

`motion_planning.py` is basically a modified version of `backyard_flyer.py` that leverages some extra functions in `planning_utils.py`. It should work right out of the box.  Try running `motion_planning.py` to see what it does. To do this, first start up the simulator, then at the command line:
 
```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python motion_planning.py
```

You should see the quad fly a jerky path of waypoints to the northeast for about 10 m then land.  What's going on here? Your first task in this project is to explain what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script, and how the functions provided in `planning_utils.py` work. 

### Step 7: Write your planner

1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!


### Explain the Starter Code 

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

#### motion_planning.py 

The script of motion_planning.py contains a basic planning implementation between the drone and computer that including
- the setup of states for drone 
- path planning and navigatin
- state position, trasitioning and callback   

#### planning_utils.py

The script of planning_utils.py provides 
- Grid creation
- Calculates the optimal path using A* 
- Prune the path using collinearity check.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

- The colliders.csv file contains the data of initial lat and lon.

``` 
        # Read lat0, lon0 
        filename = 'colliders.csv'
        with open(filename) as f:
            tmp = f.readline()
        lat0,lon0=np.float64(tmp.split(',')[0].replace('lat0','')), np.float64(tmp.split(',')[1].replace('lon0',''))
        print('lat0:{},lon0:{}'.format(lat0,lon0))
        self.set_home_position(lon0,lat0, 0) 
 ```
 


#### 2. Set your current local position
- Retrieve the current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. 
- Use the utility function global_to_local() to convert to local position using self.global_home
 

 ``` 
        # TODO: convert to current local position using global_to_local()
        global_position_lon,global_position_lat,global_position_alt = self.global_position[0],self.global_position[1],self.global_position[2]
        start_loc = global_to_local(self.global_position,self.global_home)
```



#### 3. Set grid start position from local position
```
        grid_start = (int(np.ceil(start_loc[0] - north_offset)) , int(np.ceil(start_loc[1] - east_offset)))
```

#### 4. Set grid goal position from geodetic coords
```
    # Define Parser 
    parser.add_argument('--global_goal_lon', type=str, default='-122.397755', help="Goal position longitude")
    parser.add_argument('--global_goal_lat', type=str, default='37.793839', help="Goal position latitude")
    parser.add_argument('--global_goal_alt', type=str, default='0', help="Goal position altitude")

    # Parse the global_goal_position
    
    global_goal_position  = ( validate_arg(key='global_goal_lon')  , validate_arg(key='global_goal_lat') , validate_arg(key='global_goal_alt') )
    drone = MotionPlanning(conn, global_goal_position=global_goal_position)

    # Set the global_goal_position
    goal_loc = global_to_local(self.global_goal_position, self.global_home)
    grid_goal = (int(np.ceil(goal_loc[0] - north_offset)), int(np.ceil(goal_loc[1] - east_offset)))

```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
- Added To ActionEnum
```
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    # for diagnal
    SOUTH_EAST = (1, 1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))
```


- Added to valid_actions
``` 
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    return valid_actions
```

#### 6. Cull waypoints 
```
- Use a collinearity check for path pruning
- Added to planning_utils


def prune_path(path, epsilon=1e-6):
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def check_collinearity(p1, p2, p3):
        return abs(np.linalg.det(np.concatenate((p1, p2, p3), 0))) < epsilon

    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        collinear = check_collinearity(point(pruned_path[i]), point(pruned_path[i + 1]), point(pruned_path[i + 2]))
        if collinear:
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path

```



#### 7. Execute the flight 
It works!
```
  #Run from command line: 
    >python motion_planning.py --global_goal_lon -122.397755 --global_goal_lat 37.793839

    or with pre-defined params
    
    >python motion_planning.py

![Video](./videos/zigzag.mp4)
  # Flight video: https://youtu.be/ztrvaW3ITFY 

```

