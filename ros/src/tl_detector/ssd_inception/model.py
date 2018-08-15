from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

class Model(object):
    def __init__(self, model_path):
        print('debug', model_path)
        self.model = self.load_graph(model_path)

        tf_config = tf.ConfigProto(device_count={'gpu': 0}, log_device_placement=True)
        tf_config.gpu_options.allow_growth = True
        with self.model.as_default():
            self.session = tf.Session(config=tf_config, graph=self.model)
            # TODO: are these correct for our models? most prob not. getinfo from Damian
            # TODO: I assume it will be different for site and sim.
            # then better to write two "get_classification" functions
            # Damian comment: These are correct. They will be the same for Real & Sim Models
            self.image_tensor = self.model.get_tensor_by_name('image_tensor:0')
            self.detect_boxes = self.model.get_tensor_by_name('detection_boxes:0')
            self.detect_scores = self.model.get_tensor_by_name('detection_scores:0')
            self.detect_classes = self.model.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.model.get_tensor_by_name('num_detections:0')

    def detect_traffic_light(self, image):

        # Expand dimensions since the model expects image to have shape : [1, None, None,3]
        image_np_expanded = np.expand_dims(image, axis=0)

        (boxes, scores, classes, num) = self.session.run([self.detect_boxes, self.detect_scores, self.detect_classes, self.num_detections],
                                                 feed_dict={self.image_tensor: image_np_expanded})

        num = int(num)

        detected_lights = []
        for i in range(0, num):
            detected_light = TrafficLight.UNKNOWN
            if scores[0][i] > 0.4:  # TODO: tune this threshold
                if int(classes[0][i]) == 1:
                    detected_light = TrafficLight.GREEN
                elif int(classes[0][i]) == 2:
                    detected_light = TrafficLight.RED
                elif int(classes[0][i]) == 3:
                    detected_light = TrafficLight.YELLOW
                else:
                    detected_light = TrafficLight.UNKNOWN

            detected_lights.append(detected_light)

        # TODO: make sure the enumeration used in models are the same as in styx_msgs/TrafficLight
        if len(detected_lights) > 0:
            # print('debug: detected lights: ', detected_lights)
            return detected_lights[0]
        else:
            return TrafficLight.UNKNOWN

    # CODE FROM https://github.com/alex-lechner/Traffic-Light-Classification
    def load_graph(self, graph_file):
        # """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph