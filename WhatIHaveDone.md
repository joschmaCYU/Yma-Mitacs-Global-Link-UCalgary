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
