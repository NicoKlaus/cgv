### Usage ###

Use the controllers to target boxes
Grab targeted boxes by touching the trackpad

In edit mode

create boxes by pushing the trigger
point at a box and press the button on the grip to set this box as template for new boxes.
change the extents of grabbed boxes by pushing the trackpad left down or right.
delete boxes by pressing up on the trackpad.

### Log files ###

log entries found in multiple files types

controller-id: 	identifies controller
grab-number:	identifies grab operation, is counted for every controller individualy
time:			time since start in milliseconds		
				
the grab number increases only after releasing a box from a grab
thus entries with the same combination of grab-number and controller-id refer to the same 
cycle of grabing a box, moving it and finally releasing the box

# Box trajectory #

btrj file format:
<controller-id> <grab-number> <time> <box-index> <box-translation> <box-orientation>

box-translation: 3d cartesian coordinates of the boxes center
box-orientation: rotation applied to the box as quaternion

# Controller trajectory #
The controller trajectory is stored as a series of three dimensional cartesian coordinates.
the orientation stored as quaternion
ctrj file format:
lines are built after the scheme
<controller-id> <grab-number> <time> <position> <orientation>

example:
0 -1 1.00594e+07 0.119393 0.764416 0.267091 0.0807427 0.39929 -0.110382 0.906567

used controller: 0
grab_number : -1 which means nothing was grabbed at that point
timestamp: 1.00594e+07 ms after start of the application
position: x=0.119393 y=0.764416 z=0.267091
orientation: r=0.0807427 x=0.39929 y=-0.110382 z=0.906567 #is given as quaternion


entries with the same combination of grab-number and controller-id refere to the same grab operation

# VR-Events #
Pressing buttons and moving the vr controllers triggers events inside the application.
These events can be recorded into vrcr files.
For every recorded event there is a line describing the event.
The format of the strings come from event::stream_out(std::ostream& os) in event.cxx.
At the moment there is not a method to convert these back into cgv::gui::event objects.

vrcr file format:

<timestamp> "<event>"
example for a pose event:
1.00594e+07 "pose[10059.3] {VR}x(0.68737,0.68737,-0.724637);y(-0.305784,-0.885353,-0.350208);z(-0.658801,0.462305,-0.593511);O(-0.140597,0.776636,0.310234);<1:-1>*77FB4020*"


# Save and Load Box Configurations #

Box configurations can be stored in a file by pressing the save button in the category "movable boxes"
The load button reads a box configuration from a file.
Both buttons open a dialog.

Each line in the resulting .vrboxes file stands for a box in the scene
BOX <box.min_p> <box.max_p> <trans> <rot> <col>

