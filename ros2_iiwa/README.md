##NOTE PER RUNNARE IL MONDO CON GAZEBO E CON L'ARUCO TAG##
Punto 2a source
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src
```
Per lanciare il mondo con il robot lancia questo
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true
```
Per vedere il tag lancia questo 
```
$ ros2 run rqt_image_view rqt_image_view
```
Per la pose del aruco lancia questo
```
$ ros2 topic echo /aruco_single/pose
```


