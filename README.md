**Инструкция по запуску алгоритмов серии Bug**






**Необходимые преустановленные файлы и пакеты:**

1. Установленная система для работы с ROS Noetic и созданный workspace(http://wiki.ros.org/ROS/Tutorials) 

2. Пакеты для работы с turtlebot3
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

3.Пакеты для работы в gazebo с turtlebot3(все туториалы должны быть для ros noetic)
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

4. Python3 для запуска скриптов, написаных на нём





**Установка пакета с алгоритмами:**

1. Скачать архив с пакетом src

2. Разархивировать его в catkin_ws
(вместо catkin_ws должно быть название Вашего воркспейса)

3. Проверить, что все файлы в установленном пакете executable

_Для этого заходим в scripts и в Properties/permissions ставим галочку около allow executing file as program
Аналогично повторяем с CMAKE, xml, setup.py файлами в самом пакете, bug_alg, launch файлами в launch и файлом по адресу ~/catkin_ws/src/bug_alg/src/bug_alg/__init__.py _

Или проще выдать права этим файлам, написав в терминале:
```
cd ~/catkin_ws/
chmod +x src/bug_alg/CMakeLists.txt
chmod +x src/bug_alg/package.xml
chmod +x src/bug_alg/setup.py
chmod +x src/bug_alg/launch/*.launch
chmod +x src/bug_alg/scripts/*.py
chmod +x src/bug_alg/src/bug_alg/__init__.py
```

4. Открываем terminal и пишем команды 

```
cd ~/catkin_ws/

catkin_make

source devel/setup.bash
```


5. Иногда возникают проблемы с catkin_make и приходится повторять 4 шаг несколько раз


6. Скачать архив с пакетом Classical_Labs и разархивировать рядом с catkin_ws

7. В файле catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/small.world заменить username на актуальное имя пользователя



**Средства работы с мирами Газебо**

Для миров из папки со средами запуска достаточно будет переместить launch файл в ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch и заменить <worldname>.world в самом файле на расположение мира газебо у себя в системе

Проверьте, что после скачивание папки адрес dae файла внутри файла world совпадает с расположение соответствующего dae файла в системе

Для этого нам понадобится заранее установленный пакет для работы в gazebo с turtlebot3

1. Заходим в папку ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

2. Создаём файл <filename>.launch
 Копируем туда код, заменив <worldname>.world на расположение мира газебо у себя в системе
 
```
 
 <launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="1.5"/>
  <arg name="y_pos" default="1.5"/>
  <arg name="z_pos" default="1.0"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="<worldname>.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model turtlebot3 -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
</launch>


<arg name="x_pos" default="1.5"/>
<arg name="y_pos" default="1.5"/>
<arg name="z_pos" default="1.0"/>
```



Изменяя аргументы в этих трёх строчках можно менять координаты спавна робота



**ВАЖНО:** Проверьте, что после скачивания world файла адрес соответствующего dae файла внутри совпадает с расположением этого же dae файла в системе

3. Откройте терминал и напишите 
`export TURTLEBOT3_MODEL=burger`

Затем
`roslaunch turtlebot3_gazebo <filename>.launch `
Например, для запуска мира turtlebot3_world.world:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
После этого загрузится Ваш собственный мир газебо

Чтобы не писать каждый раз эти команлды можно написать скрипт на bash'е и расположить его в папке catkin_ws, дать название файла, например, "start.bash".

start.bash
```
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo <filename>.launch
```

Для запуска карты:
```
cd ~/catkin_ws/
bash start.bash
```




**Запуск алгоритмов**

В новом терминале
`export TURTLEBOT3_MODEL=burger`
1. Открыть консоль и прописать
`roslaunch bug_alg algname.launch des_x:=6 des_y:=6`

algname - в зависимости от желаемого алгоритма
Например, для запуска алгоритма bug1:
```
export TURTLEBOT3_MODEL=burger
roslaunch bug_alg bug1.launch des_x:=0.7 des_y:=0.7
```

Меняя значения des_x и des_y можно изменять координаты точки, до которой необходмио будет доехать роботу

Алгоритм можно запускать с помощью Makefile'а. Makefile должен располагаться в папке catkin_ws
Makefile :
```
bug1:
	roslaunch bug_alg bug1.launch des_x:=6 des_y:=6

bug2:
	roslaunch bug_alg bug2.launch des_x:=-1.1 des_y:=-1.1

class1:
	roslaunch bug_alg class1.launch des_x:=2 des_y:=-1

distbug:
	roslaunch bug_alg distbug.launch des_x:=3 des_y:=0
```

Запустить алгоритм:
```
cd ~/catkin_ws/
make {алгоритм}
```

**Тестирование**


Предлагается тестировать на двух типах карт:
1. Карта с замкнутыми препятствиями turtlebot3_world
Файл turtlebot3_world testcases


2. Лабиринт small
Файл small_world testcases
