# FCND-Controls
3rd Project - Control of a 3D Quadrotor


## Ptyhon Part

![Photo_1](./image/Photo_1.png)
---

In this part, the nested low-level controller needed to achieve trajectory following is explained. The controller is separated into five parts:

 - body rate control
 - reduced attitude control
 - altitude control
 - heading control
 - lateral position control
 
 The block diagram of this contoller is here
 
 ![Photo_2](./image/Photo_2.png)

* **1- Body Rate Control ( body_rate_control() )**
```py
print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
```
* **2- Reduced Attitude Control ( roll_pitch_control() )**
```py
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
```    
* **3- Altitude Control ( altitude_control() )** 
```py
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

```  
* **4- Heading Control ( yaw_control() )**
```py    
 grid_start = (-north_offset, -east_offset)
       
 grid_goal = (-north_offset + 10, -east_offset + 10)
```

* **5- Lateral Position Control ( lateral_position_control() )**
```py    
 grid_start = (-north_offset, -east_offset)
       
 grid_goal = (-north_offset + 10, -east_offset + 10)
```
