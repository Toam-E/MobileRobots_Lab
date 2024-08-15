# ECE 046214 Mobile Robots Project

## Problem description
We have a planned trajectory we've found (using KinoRRT or PRM) and we want to follow it w/ a controller that minimizes the error from it.
This can be done using the Pure Pursuit controller. However, if some unplanned obstacles are places along the path, we will hit them if our robot 
doesn't sense and correct it path locally.
We try to solve the obstacle/crowd avoidance problem by incorporating MPC (Model Predicitive Control) to it.

## Pure Pursuit only example
![Pure Pursuit only](pp_only_2_plan_15-08-2024_09-41-42.gif)
## Model Predicitive Control + Pure Pursuit (MPC + PP) example
![MPC (Model Predictive Control + Pure Pursuit](mpc_pp_2_plan_15-08-2024_09-50-06.gif)
