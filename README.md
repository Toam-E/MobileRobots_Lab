# ECE 046214 Mobile Robots Project

## Acronyms 
PP - Pure Pursuit  
MPC - Model Predictive Control  
RRT - Rapidly-exploring Random Tree  
Kino RRT - dynamic version of RRT  


## Problem description
We have a planned trajectory we've found (using KinoRRT) and we want to follow it w/ a controller that minimizes the error from it.  
This can be done using the Pure Pursuit controller. However, if some unplanned obstacles are places along the path, we will hit them if our robot   
doesn't sense and correct it path locally.  
We try to solve the obstacle/crowd avoidance problem by incorporating MPC (Model Predicitive Control) to it.  

In the examples below we can see the difference in real time obstacles avoidance.  

## Pure Pursuit only example
![Pure Pursuit only](src/pp_only_scaled_down_plan_18-08-2024_01-45-10.gif)
## Model Predicitive Control + Pure Pursuit (MPC + PP) example
![MPC (Model Predictive Control + Pure Pursuit](src/mpc_pp_scaled_down_plan_18-08-2024_01-40-54.gif)

Legend:  
balck - free space  
yellow - obstacles (originally when global planning was done)  
blue - added real time obstacles over the planned trajectory  
cyan line - planned trajectory (by some planning algorithm like PRM or RRT. We used Kino RRT)  
light purple line - the actual trajectory (when we use PP only or MPC+PP)  
purple arc - the lidar like sensor model when it doesn't hit an obstacle  
red arc - the lidar like sensor model when it does hit an obstacle  
orange rectangle - local pllaner search area when we look for a local alternative to bypass the obstacle in our way using local start and goal and running  
red line - the selected local KinoRRT path found when we search locally  
light green ball - the PP look ahead point on the originally planned trajectory  
big green ball - start point  
big red ball - goal point  
