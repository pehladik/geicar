# PathPlanner

## Algorithm

The `algo.py` file contains the code for the A-star algorithm which is used for path-planning. Only works on grid.

TODO: Try other solutions (Hybrid A-star, Machine Learning, RTT*)

## GUI

The `gui.py` file contains the code of the interface used to compute the A-star algorithm. Use the library `pySimpleGui`.

You have to create the grid first, then create a start point by clicking left one time (**green square**) and an endpoint by clicking left two times (**red square**). Click Right create obstacles (**black squares**)

By default the `Create_Grid_Radar` button use the text file `obstacles_real.txt` to create the grid. This file contains data from the real radar that we saved.
You can use the GUI with ROS.

## What next ?

There is a lot to do on the path planning :

    -Dynamic path planning
    -Use GPS data to follow the trajectory computed
    -Take into account that the path computed is for one orientation of the vehicle
    -If you decide to use the A-Star, add other constraints
    -Use speed and size of obstacles to create the grid

## Recommandation
I recommend checking in ROS for global and local path planner because there are built-in functions for avoidance trajectories
