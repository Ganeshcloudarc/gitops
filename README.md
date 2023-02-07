# Boson Motors patrol application
These are set of packages for patrolling application on boson vehicles.


## Getting started

- TODO 

### Build the repository on NRU
Open a terminal, clone the repository, update the dependencies and build the packages:
```
cd ~/catkin_ws/src
git clone https://github.com/bosonrobotics/autopilot_boson.git
cd ../
pip3 install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
source ./devel/setup.bash
```

### Run the patrolling app
To save the path:

    $ roslaunch autopilot save_path.launch 

To replay the path(patrolling):

    $ roslaunch autopilot patrol.launch
    
