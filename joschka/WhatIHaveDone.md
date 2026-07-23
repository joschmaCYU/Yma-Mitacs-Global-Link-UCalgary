I am working on a quadruped that will do search and rescue missions. 

What I need to do for now : setup lidar, then publish a grid to the /lidar topic (PointCloud or Float32MutliArray) the grid can change in function of the policy.
# Day 1
I familarised my self with all the pdfs they gave me.
Meet Dr. Alex and all the other student. Get an idea what I shout do: setup the lidar, connect it to /lidar in ros and then use the "neck" to map the environement in 3d.
Connected the lidar (ouster OS0 64) with ethernet cable to pc.

# Day 2
Setup Docker and ros noetic to work (display issues)
Did not update the lidar firmware just used an old version of https://github.com/ouster-lidar/ouster-ros.git
Can see the lidar in gz (see video 30/06/2026)
Launch with `mitacs_ouster.launch`

# Day 3
I have made the lidar publish a grid of 1.6 x 1m with (rows x cols) 17 x 11 points with a resolution of 0.1. (see video 02/07/2026). I can update if the policy is croutch (it adds a point on the back so that it can see the ceiling). It needs to see in front (from 0 to 1.6m). So that it ignores left, right and his back. I need to becarfull that the the points that it gets are always perfectly parrallel points to the ground (because the RL is trained so).

To get the grid with all the points that I'm getting from lidar I use this formula : $i_{row} = \lfloor \frac{x - x_{min}}{x_{max} - x_{min}} \times N_{rows} \rfloor$  
x (in meters) the actual points that the lidar is seeing. $x_{min}$ and $x_{max}$ (m) the "seeing" zone. $N_{rows}$ number of rows. $i_{row}$ the number of column.

Corrected so that the lidar think it always faces the ground. You can visualise the Float32MutliArray on /lidarvisu and see with the grid with more points on lidarcap

## Mesure that the lidar doesn't break the limit of 1m/s at 
### Alpha 0.2
=== RÉSULTATS DE L'ANALYSE DU BRUIT (Sans les 0.0 purs) ===
Transitions analysées            : 42914
Transitions ignorées (Vides)     : 50399
----------------------------------------
Vitesse de saut max (Positif)    : 1.141 m/s
Vitesse de saut max (Négatif)    : -2.060 m/s
Vitesse moyenne de fluctuation   : 0.015 m/s
----------------------------------------
❌ TEST ÉCHOUÉ : Le seuil de +- 1 m/s a été dépassé !
   -> Nombre de dépassements : 14 fois (0.03% des vraies mesures)

### Alpha 0.25 (appart)
=== RÉSULTATS DE L'ANALYSE DU BRUIT (Sans les 0.0 purs) ===
Transitions analysées            : 38980
Transitions ignorées (Vides)     : 4932
----------------------------------------
Vitesse de saut max (Positif)    : 1.371 m/s
Vitesse de saut max (Négatif)    : -1.333 m/s
Vitesse moyenne de fluctuation   : 0.016 m/s
----------------------------------------
❌ TEST ÉCHOUÉ : Le seuil de +- 1 m/s a été dépassé !
   -> Nombre de dépassements : 19 fois (0.05% des vraies mesures)

### Alpha 0.50
=== RÉSULTATS DE L'ANALYSE DU BRUIT (Sans les 0.0 purs) ===
Transitions analysées            : 42914
Transitions ignorées (Vides)     : 50399
----------------------------------------
Vitesse de saut max (Positif)    : 3.146 m/s
Vitesse de saut max (Négatif)    : -5.431 m/s
Vitesse moyenne de fluctuation   : 0.039 m/s
----------------------------------------
❌ TEST ÉCHOUÉ : Le seuil de +- 1 m/s a été dépassé !
   -> Nombre de dépassements : 164 fois (0.38% des vraies mesures)

### Alpha 1
=== RÉSULTATS DE L'ANALYSE DU BRUIT (Sans les 0.0 purs) ===
Transitions analysées            : 41010
Transitions ignorées (Vides)     : 52303
----------------------------------------
Vitesse de saut max (Positif)    : 8.242 m/s
Vitesse de saut max (Négatif)    : -7.312 m/s
Vitesse moyenne de fluctuation   : 0.092 m/s
----------------------------------------
❌ TEST ÉCHOUÉ : Le seuil de +- 1 m/s a été dépassé !
   -> Nombre de dépassements : 792 fois (1.93% des vraies mesures)

At alpha 0.20 we get 0.2s to react to 90% of obstacle and 0.08s as mean reaction time. 0.2s corresponds to 50Hz which is the speed at which the sim moves. So we will take alpha = 0.20

# Day 4 (03/07)
Beggining to use the nucleo board. I have made the motor move now I want to control them and send the messages to ros.

# Day 5 (06/07)
Getting the nucleo to work with the old code. I found it I had to flash it back but now the angles wont begging computed correctly. To make the stm board work you need to unplug the encoder so they would initialise at the wring values.
There is a problem with the code idk what but see "*Motor_Performance_Analysis*" graphs but they don't make any sens to me.

# Day 6 (07/07)
Trying to get the neck to work. I tryied the default version with putty but it doesn't work. I don't get why it doesn't listen to the serial.
I undestand it's the motor driver that is faulty. The second output ins't working.

# Day 7 (08/07)
It's not the motor dirver fault. I tryied a simple move back and forward script and everything was working great.
I am reimplmenting everything on arduino IDE and it's working. 
The encoder are still not very friendly. These are absolut magnetic SPI.

It's now working but there is a lot of wigle room so eventhoug the motor moves the outside parts doesn't. So I unmounted the motor and tied the srews

# Day 8 09/07
It's working see *Working_Motor_Performance_Analysis_14.xlsx*.

I am now sending the motors info to ros. 
I had to **change some pins** because I had to find output with some clock (SCK).
I moved 

# Day 9 10/07
Today I need to combine the lidar and Orbita.
Updating ouster lidar firmware from v2.5.2 to v2.5.3
But I had to downgrade in my dockerfile the ouster-ros.
It works from my pc with ros to the nucleo that makes the lidar neck move.

I need to somehow make the lidar fit the simulation but the problem is that the lidar of the sim comes from above the robot and the real lidar is at the front of the robot. 
So I have multiple choises :
1) I map the env with slam and give the map in real time to the rl. So I have to send the zone around the robot as a grid.
2) Move the head to map a max of the env and assume that the rest is flat.

The problem is that the lidar is at the front. It can't perfectly see everything around it so it can't produce the real grid for the rl policy. I will map the env so that the robot can use it eventhoug it can't actively see it.
Question for Stefan:
1) How can I simulate the robot in a rougth env and add a lidar on it (urdf?)
    i) How to make the robot turn on it self/go somewhere
2) I will assume that every point I did not see is flat
3) Move the neck a maximum to get more info at the start

# Day 10 (13/07)
Today I am putting the quadruped into simulation. I updated the urdf to add the lidar. Then I will use [grid_map](https://github.com/ANYbotics/grid_map) to construct a height map of the environement. 
Launch with `elevation_mapping.launch`

# Day 11 (14/07)
For now I spawn the robot urdf and I publish my grid map on `/elevation_mapping/elevation_map`. I can now see elevation points on rviz. I publish the grid on `/lidar`.
1) I had to adjust the pid gains.
2) The order for the rl is `["FL_HAA", "FR_HAA", "HL_HAA", "HR_HAA", "FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE", "FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE", "HL_AFE", "HR_AFE"]`
3) The joint position actions scale is 0.5
4) The default angles are: "FL_HAA": 0.0, "FR_HAA": 0.0, "HL_HAA": 0.0, "HR_HAA": 0.0, "FL_HFE": 0.4102, "FR_HFE": 0.4102, "HL_HFE": -0.6981, "HR_HFE": -0.6981, "FL_KFE": -1.2716, "FR_KFE": -1.2716, "HL_KFE": 1.676, "HR_KFE": 1.676, "HL_AFE": -1.7219, "HR_AFE": -1.7219

Test with just the flat policy.

# Day 12 (15/07)
Trying to setup Luis simulation to see what he did. The commands are : 
`roslaunch quadruped_gazebo gazebo.launch
roslaunch quadruped_gazebo spawn_control.launch
roslaunch quadruped_control bringup_rl.launch`
I copied Luis files and you can launch the luis config with lidar with `roslaunch mitacs luis_bringup.launch`
I took his code and added the mapping part over it so that the robot creates an elevation map.

The pipe line for the simulated lidar is :
1) URDF : <sensor type="ray" name="ouster_sensor"> combined with <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_velodyne_laser.so"> which is published on `/ouster/points`
2) You need to create the TF tree which is done with odom_to_tf.py and robot_state_publisher publishes to the `world` TF
3) With the /ouster/points you can calculate the world position of each point
4) Creation of the 2.5d grid which is published on /elevation_mapping/elevation_map

If you want the points to go farther you have to modify `length_in_x` in elevation_mapping.yaml

# Day 13 (16/07)
Luis configured the pid for just walking and for the flat policy.
Tuning for rough policy.

# Day 14 (17/07)
I think I have a good pid. I created a simple script to move each part individualy `manual_joint_tuner.py`.
I have one leg that is up I don't know why.

I have to tune tow legs front and back because robot is mirror for this I can make the robot go up and down and then for haa make one leg move at a time on the side but other legs a bit out so that the robot doesn't fall.

# Day 15 (20/07)
I made `stance_tuner.py` I could tune my pid for the legs `p: 200.0  i: 0.0  d: 4.0` for the shoulders (haa) : `p: 250.0  i: 0.0  d: 1.0`, for afe `p: 150.0  i: 0.0  d: 1.0`. 
See *HL_HFE_KFE.png*, *FL_HFE_KFE.png*, *HL_AFE.png* and *HL_HAA.png*.
But the robot still falls when I tell him to go forward. I use the same urdf that was used for the rl.
**I undestood I gave the wrong start positions.**

# Day 16 (21/07)
Still not walking strait.
I visualised the output of *joint_positions_rough_with_flat_terrain.csv* with `replay_policy.py` to see how the robot should walk.
I also visualised the output of the flat and the rough policy with the same data `joint_positions.csv` see *compareson_flat_rough_same_data* folder.
**I put the same data but I don't have the same output for the same policy**

# Day 17 (22/07)
I am trying to pin down what is the problem I think that I don't have the same policy as in isaac sim because why else would I have different output with same input.
I have the same policy as in isaac sim (2ae2ad6d363eb7d1a739e867d2a003f9 2025-09-15_13-06-21_fixed-slope-2.onnx) md5 checksum
**I had the joint in the wrong order** now it's working. Like I have the same output with the same input for *flat* and *rough*! It also works with lidar data set to 0. So did I show that the robot will work whithout a lidar on a flat terrain? See `same_input&output.png`.
But it still doesn't work in gz with rough policy

# Day 18 (23/07)
I will use florent package.
I broke in multiple piceses the urdf: 
- `continuo.urdf.xacro`: the main file that includes the other files
- `materials.xacro`: def the materials
- `sensors.xacro`: link and joints for the lidar and imu
- `transmissions.xacro`: for the transmissions that makes the link between the URDF joints and the PIDs
- `legs.xacro`: All the legs stuff (inertial, visual, collision, link, joint)
