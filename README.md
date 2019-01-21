# Path_planning_and_control_python


Reference:
https://github.com/lucianzhong/PythonRobotics

Control folder:
 1.move_to_pose_PID.py  Using PID controller to move the vehicle to a goal position.The PID controller controls both the velocity and steering angle
 
 2.pure_pursuit.py 预瞄法，找到路径上的在当前位置的前面一点，计算出当前的需要的方向盘转角，PI controller
 
Frenet_optimal_trajectroy folder:
  1. frenet_optimal_trajectory: convert the x-y coordinates to s-d coordinates
                                sampling in road_width & sampling in time period & sampling in longitudinal velocity
                                get the min cost sampling set
                                from the sampling set to create the 3rd or 4th polynomial curve which has a min jerk value
                                using the curve and sampling time period to caluculate the next-time s,d,ds/dt, dd/dt
                                convert s,d,ds/dt, dd/dt to x,y,dx/dt,dy/dt
                                send the next-time x,y,dx/dt,dy/dtx,y,dx/dt,dy/dt to low-level controller
   
  2. cubic_spline_planner: from points to generate a 3rd curve
  
MPC_vehicle_control:
        control a car's velcity and steering angle to reach a goal point
        MPC can be formed as a standard formate
        The solver using decent gradient method to choose control inputs and solve the MPC problem
        
A_star.py:  g=f+h

Dynamic_window_approach: another form of sampling controller 

RRT: the sampling method
