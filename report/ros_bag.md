# ROS bag

## Overview

There are 3 ROS bags provided to completed the project.

* just_traffic_light.bag
* loop_with_traffic_light.bag
* reference.bag

Two of 
```python
bag=rosbag.Bag('../data/just_traffic_light.bag')
topics = bag.get_type_and_topic_info()[1].keys()
types = []

for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
    types.append(bag.get_type_and_topic_info()[1].values()[i][0])
```

```
Topics: ['/image_raw', '/current_pose']
Types: ['sensor_msgs/Image', 'geometry_msgs/PoseStamped']
```

