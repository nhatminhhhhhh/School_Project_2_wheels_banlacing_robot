# School_Project_2_wheels_banlacing_robot
I made 2 wheels banlacing robot for my automation class

I use 2 DC Motor with gearbox, MPU6050 and ESP32 Dev Module  
I use Pitch Angle of MPU6050 to control the robot angle.  
In my case, the motor speed when go forward and go back is different. So I have to modify the speed -> PID Output for go back is higher.  
I think, the 2 wheels banlacing robot use high Ki parameter, to avoid "overshoot" 
