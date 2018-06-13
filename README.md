# Węzeł do obsługi Optitrack oraz IMU

### 1. Pobranie repozytorium Git:
```
$ git clone --recurse-submodules https://github.com/kcwian/optitrack.git
```
### 2. Kompilacja węzła
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
### 3. (Opcjonalne) Uruchomienie wyłącznie węzła Optitrack (mocap_ros2) w celu sprawdzenia poprawności działania

#### Terminal 1
```
$ roscore 
```
#### Terminal 2
```
$ source devel/setup.bash
$ rosrun mocap_ros2 mocap_ros2 -l 192.168.100.xxx -s 192.168.100.xxx
```
gdzie:  
**-l** - lokalny adres IP  
**-s** - adres IP serwera Optitrack  
#### Terminal 3
```
$ rostopic list
$ rostopic echo /mocap_node/optitrack0
```

### 4. Uruchomienie węzła Optitrack wraz z czujnikami IMU (xsens, microstrain, adis, low_cost)

#### Należy zmodyfikować plik opti_launch.launch w folderze sensors/launch:
-	args = "-l 192.168.100.xxx -s 192.168.100.xxx"

gdzie:  
**-l** - lokalny adres IP  
**-s** - adres IP serwera Optitrack  

#### Wywołanie pliku launch:
```
$ roslaunch sensors opti_launch.launch
```
#### Wyświetlenie listy dostępnych topic'ów:
```
$ rostopic list
$ rostopic echo /mocap_node/optitrack0
```
### 5. Zapisywanie danych do pliku .bag
```
$ roslaunch opti_save.launch
```
