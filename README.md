# Węzeł do obsługi Optitrack oraz IMU

### Pobranie repozytorium Git:
```
$ git clone --recurse-submodules https://github.com/kcwian/optitrack.git

```
### Kompilacja węzła
```
$ catkin_make
$ source devel/setup.bash
```
### (Opcjonalne) Uruchomienie wyłącznie węzła Optitrack (mocap_ros2) w celu sprawdzenia poprawności działania

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

### Uruchomienie węzła Optitrack wraz z czujnikami IMU (xsens, microstrain, adis, low_cost)

1. Należy zmodyfikować plik opti_launch.launch w folderze sensors:
-	args = "-l 192.168.100.xxx -s 192.168.100.xxx"

gdzie:
**-l** - lokalny adres IP 
**-s** - adres IP serwera Optitrack 
2. Wywołanie pliku launch:
```
$ roslaunch sensors opti_launch.launch
```

### Wyświetlenie listy dostępnych topic'ów:

```
$ rostopic list

```
### Zapisywanie danych do pliku .bag
```
$ roslaunch opti_save.launch
```

