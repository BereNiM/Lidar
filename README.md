# LidarPlugin
Rover plugin

Este plugin permite mover los joints de dos actuadores Dynamixel AX-12A, así como ajustar las variables de sus controladores PID. Para ejecutarlo, clone este repositorio en su PC e inicie ROS con la instrucción `roscore`. En otra consola inserte 
```
cd lidar_plugin/build/
gazebo --verbose ../lidar.world
```
En otra terminal.
```
cd lidar_plugin/build/
./pos 0 0.2
```
Donde el 0 representa el joint 0 y el 0.2 la posición que deseamos tome el joint. Si quisieramos, por ejemplo, mover el joint 1 a una posición de 0.3; entonces la instrución sería `./pos 1 0.3`.
![image](https://user-images.githubusercontent.com/42397059/119287419-143b6b80-bc0c-11eb-8f8b-62b2898c4a34.png)


Por otro lado si lo que deseamos es ajustar las variables de los controladores PID abrimos una nueva consola y copiamos las siguientes líneas.
```
cd lidar_plugin/build/
./pid1 10 6 0
```
Para el PID del joint cero usamos ./pid1 y para el PID del joint 1 usamos ./pid2. 

Cada que modifique la posición de los joints o los valores de los controladores usted podrá ver, en la termnal donde se está ejecutando gazebo,los valores reales de los joints así como los valores que acaba de asignar a los PID.

![image](https://user-images.githubusercontent.com/42397059/119287756-bfe4bb80-bc0c-11eb-8785-397f70a81071.png)
