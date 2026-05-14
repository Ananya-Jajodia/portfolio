+++
title = "Lab 12: Path Planning and Execution"
description = "In this lab, we use a bayes filter for localization and PID for control to follow a path"
date = "2026-04-27"

[taxonomies]
tags = ["ece5160", "Artemis RedBoard Nano", "C Programming", "Bayes Filter"]
+++

# Lab 12: Path Planning and Execution
In this lab, we use PID and a Bayes filter to move through a series of waypoints in the environment from Lab 12. This lab involved pathing to a way point, executing a control sequence, then localizing to determine the robots new location. 

## Control and Navigation
Our goal was to follow the path below, starting in the botton left hand corner and moving around the map from there.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab12/goal.png?raw=true" alt="lab 12 goal path">


To do this, we need to attempt to move to next waypoint and then localize once we arrive there. I broke down the steps needed in the state diagram below.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab12/state.png?raw=true" alt="distance v time for meep">

Since we already established how to localize in Lab 12, we needed to eestablish a method for moving between way points in this lab. I decided to use PID to achieve the exact facing angle for each move, then send a straight motor command to drive to the point based on the distance that Meep (my robot) needed to travel. To figure out the distance Meep was expected to travel over time, I used data I had gathered from Lab 7 of Meep traveling when sent a 58.8% duty cycle. 



I used this to make an equation to how far Meep was expected to travel when moving for a specific amount of time, then used fsolve to solve this equation when given a desired distance. To extrapolate to farther distances, I used a simple linear function to model the tail end of the graph.

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab12/dt.png?raw=true" alt="distance v time for meep">

<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab12/fitted.png?raw=true" alt="distance v time for meep">

After trying this equation and the linear tail mapping in real life, I adjusted the values to get the move code below:

```python 
# Fitted coefficients
a = -2.982553e-07
b = 1.613218e-03
c = -1.112209e-01
d = -8.618575

m = 2.7109512000786133
b = -1599.5880564697404

# Distance model
def distance_model(t):
    return a*t**3 + b*t**2 + c*t + d

# Solve for time given desired distance
def time_for_distance(desired_distance, initial_guess=500):
    # use linear function for large distances
    if desired_distance > 1000:
        return ((desired_distance - b) / m)/2

    func = lambda t: distance_model(t) - desired_distance
    # Solve
    try:
        t_solution = fsolve(func, initial_guess)[0]
    except:
        t_solution = ((desired_distance - b) / m)
    return t_solution/2
```

I also needed a way to calibrate the angle in the world frame to the facing angle from Meep's perspective. I desided to just grab the facing and when I set Meep up at 0 degrees at the beginning of a run and use that offset to adjust all future facing angles for the rest of the run. This worked very well.

Then I executed the seqeunce below:

``` python
x_init = waypoint_list[0][0]
y_init = waypoint_list[0][1]
angle_init = 0

# Reset Plots
cmdr.reset_plotter()

# Init 
loc.init_grid_beliefs()
cmdr.plot_bel(x_init/3.281, y_init/3.281)
cmdr.plot_gt(x_init/3.281, y_init/3.281)
belief = [x_init, y_init]

for i in range(1, len(waypoint_list)):
    cmdr.plot_gt(waypoint_list[i][0]/3.281, waypoint_list[i][1]/3.281)
    # calc distance to next point
    targ_dist = np.sqrt((belief[0] - waypoint_list[i][0])**2 +  (belief[1] - waypoint_list[i][1])**2) 
    
    # calc angle to next point
    targ_ang = np.degrees(np.atan2(waypoint_list[i][1]/3.281 - belief[1], waypoint_list[i][0]/3.281 - belief[0])) #degrees
    real_targ = wrap_angle(-targ_ang + imu_offset) # fix to be accurate according to Meep's IMU
    ble.send_command(CMD.SET_ANGLE, f"{real_targ}") # sets PID target angle
    
    # I gave Meep 5 seconds to get to the correct angle. Meep's PID is pretty snappy so this was plentyof time for it
    ble.send_command(CMD.PID_START, "1.9|0.1|0.0")
    time.sleep(5)
    ble.send_command(CMD.PID_STOP, "")
    
    # send move command
    t_needed = time_for_distance(targ_dist*1000) #ms
    ble.send_command(CMD.MOVE, f"150|{t_needed}|3")
    time.sleep(t_needed/1000 + 1)

    # Get Observation Data by executing a 360 degree rotation motion
    await loc.get_observation_data()

    # Run Update Step
    loc.update_step()
    belief = loc.plot_update_step_data(plot_data=True)
  ```


## Results
<iframe width="560" height="315" src="https://www.youtube.com/embed/WF-2fqxKTvQ?si=sIOd1_iWnAWvcNiw" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


The video above is the result of my path planning and execution. Meep does an excelent job following the desired path. When it ends up in the wrong place, it localizes to the correct place and adjusts. Since Meep wasn't coded to dodge walls, I do run into the problem of Meep path planning through walls and running into them rather than going around them. My attempt to save Meep from the wall into this video just results in it making a direct collision with the center obstacle. 


<img src="https://github.com/Ananya-Jajodia/portfolio/blob/main/content/blog/assets/lab12/path.png?raw=true" alt="meep's path">

The only time localization failed was the second to last way point. This was likely due to that area not having anough uniqie features to distinguish it from the point it ultimately localized too.

## Collaboration and Sources
I referred to [Lucca Correia](https://correial.github.io/LuccaFastRobots/Fast%20Robots%20Stuff/lab-12/)'s lab report when writing mine.