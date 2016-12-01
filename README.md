# Make robot look to audio source

You'll need a robot (may be simulated) with a `head_action` operated head.
You'll need a supported microphone by `hark_sound_source_localization`.

Install [hark_sound_localization](https://github.com/uts-magic-lab/hark_sound_localization) dependencies.

Then you can do:

```
source ~/tiago_public_ws/devel/setup.bash
mkdir -p look_sound_ws/src
cd look_sound_ws/src
git clone https://github.com/uts-magic-lab/hark_sound_localization
git clone https://github.com/awesomebytes/look_sound_source
cd ..
catkin build
source devel/setup.bash
```


# Test with TIAGo simulation

Install TIAGo simulation using [PAL Robotics instructions](http://wiki.ros.org/Robots/TIAGo/Tutorials/TiagoSimulation).

```
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel
```

```
rosrun look_sound_source look_source_audio.py
```

[![Video of the tool working on TIAGo](http://img.youtube.com/vi/R4BE1kiR_Xo/0.jpg)](http://youtu.be/R4BE1kiR_Xo)

