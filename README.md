# ar-table-pr2
Experimental stuff. 

[![Build Status](https://travis-ci.org/robofit/ar-table-pr2.svg)](https://travis-ci.org/robofit/ar-table-pr2)
 
Related repositories:
 - https://github.com/robofit/ar-table-common
 - https://github.com/robofit/ar-table-itable
 - https://github.com/robofit/ar-table-handheld

New launch - art_pr2/launch/pr2.launch - should run all nodes on pr2 (table has its own launch) - not stable, USE WITH CAUTION! Kinect and skeleton tracking must be launched separately at the moment.

### PR2 API

All topics/services etc. can be found in `/art/pr2/` namespace.

#### Manipulation

Actions:
```
/art/pr2/left_arm/pp
/art/pr2/right_arm/pp
```

Topics:
```
/art/pr2/left_arm/grasped_object
/art/pr2/right_arm/grasped_object
```

#### PR2 control

Services:
```
/art/pr2/left_arm/interaction/on
/art/pr2/left_arm/interaction/off
/art/pr2/right_arm/interaction/on
/art/pr2/right_arm/interaction/off
/art/pr2/spine/up
/art/pr2/spine/down

```

Topics:
```
/art/pr2/left_arm/interaction/state
/art/pr2/right_arm/interaction/state
/art/pr2/spine/control
/art/pr2/look_at
```
