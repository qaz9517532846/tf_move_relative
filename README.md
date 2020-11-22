# tf_move_relative
Relative motion between tf and tf with velocity planning function.

Need ROS tutorial package [tf2_learning](http://wiki.ros.org/tf2/Tutorials).

Step1. Start tf_move_relative launch file.

``` bash
$ roslaunch  tf_move_relative demo.launch
```

Step2. Start tf_move_relative client cpp file and input go to relative tf position.

``` bash
$ rosrun  tf_move_relative tf_move_relative_client
```

tf_move_relative have Trapezoid and triangle velocity planning function.

Trapezoid velocity planning.

![image](https://github.com/qaz9517532846/tf_move_relative/blob/main/image/Trapezoid_vel_planning.png)

Triangle velocity planning

![image](https://github.com/qaz9517532846/tf_move_relative/blob/main/image/triangle_vel_planning.png)

------

Copyright © 2020 ZM Robotics Software Laboratory.
