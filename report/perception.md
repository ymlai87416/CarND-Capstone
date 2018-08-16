# Perception

[//]: # (Image References)

[image1]: ../imgs/ssd_inception_real_red.PNG "ssd"
[image2]: ../imgs/ssd_inception_real_yellow.PNG "ssd"
[image3]: ../imgs/ssd_inception_real_green.PNG "ssd"
[image4]: ../imgs/yolo_simulated.png "yolo"
[image5]: ../imgs/yolo_real.png "yolo"

## Obstracle detection

This module is not implemented in this project. It is possible to subscribe to the
point cloud message available from Carla.

## Traffic light detection and light state extraction.

In this project, we drive a car in both a simulated and the real environment.

### Simulated environment
In the simulated environment, we use a trained [YOLOv3 detection system](https://pjreddie.com/darknet/yolo/) [1]
as the traffic light detector and obtains bounding boxes of traffic light (class 9). The algorithm looks for red, green
and yellow pixels in the cropped images and conclude what is the traffic light state. YOLOv3 is reported to have 35 frames per seconds
per second running on Titan X, but it is running at a slower speed in this project
because we have to process light state also.

Here is some result for the YOLOv3 detection system.

![alt text][image4]

![alt text][image5]

### Real test lot

For the real environment, we make use of the [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
and train a traffic light detection model using a pre-trained Single-shot detection [2] backed by inception v2 [3].
We retrained a pre-trained network on COCO dataset to return the bounding box along with the traffic light state, making it faster than
the approach we use in the simulated environment. Ssd-inception-v2 is reported to have 23 frames per second (42 ms) running on Titan X.

Here is some result for the SSD-inception detection system.

![alt text][image1]

![alt text][image2]

![alt text][image3]


## Reference
[1] Redmon, Joseph, and Ali Farhadi. "Yolov3: An incremental improvement." arXiv preprint arXiv:1804.02767 (2018).

[2] Liu, Wei, et al. "Ssd: Single shot multibox detector." European conference on computer vision. Springer, Cham, 2016.

[3] Szegedy, Christian, et al. "Rethinking the inception architecture for computer vision." Proceedings of the IEEE conference on computer vision and pattern recognition. 2016.

