from styx_msgs.msg import TrafficLight
from yolo3.yolo import YOLO
import numpy as np
from PIL import Image
from ssd_inception.model import Model

class TLClassifier(object):
    def __init__(self, is_site, configuration):
        #TODO load classifier
        self.is_site = is_site
        if is_site:
            self.model = Model(model_path=configuration["model_path"])
        else:
            #self.yolo = YOLO()
            self.yolo = YOLO(model_path=configuration["model_path"], anchors_path=configuration["model_anchor"],
                             classes_path=configuration["model_classes"])
            self.traffic_light_classes = 9  # TODO: hard code

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic l ight color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        if image is not None:
            image_data = np.asarray(image)

            if self.is_site:
                return self.model.detect_traffic_light(image)
            else:
                pil_image = Image.fromarray(image_data)
                # pil_image.save('/capstone/ros/test.png')
                out_boxes, out_scores, out_classes = self.yolo.detect_image(pil_image)

                detected_states = []

                for i, c in reversed(list(enumerate(out_classes))):
                    #predicted_class = self.class_names[c]
                    box = out_boxes[i]
                    score = out_scores[i]

                    if(c == self.traffic_light_classes):
                        top, left, bottom, right = box
                        top = max(0, np.floor(top + 0.5).astype('int32'))
                        left = max(0, np.floor(left + 0.5).astype('int32'))
                        bottom = min(pil_image.size[1], np.floor(bottom + 0.5).astype('int32'))
                        right = min(pil_image.size[0], np.floor(right + 0.5).astype('int32'))
                        corp_traffic_light = image_data[top:bottom, left:right, :]
                        detected_states.append(self.detect_traffic_light_state(corp_traffic_light))

                print('DEBUG: Detected states are: ', detected_states)
                if len(detected_states) > 0:
                    (_, idx, counts) = np.unique(detected_states, return_index=True, return_counts=True)
                    index = idx[np.argmax(counts)]

                    return detected_states[index]
                else:
                    return TrafficLight.UNKNOWN

        return TrafficLight.UNKNOWN

    def detect_traffic_light_state(self, image):
        def simple_thresh(img,thresh=(0, 255)):
           #apply only thresholding on image plane and return binary image
            binary = np.zeros_like(img)
            binary[(img > thresh[0]) & (img <= thresh[1])] = 1
            return binary

        rows = image.shape[0]

        #thresholding on red channel
        red_channel = image[:,:,0]
        r_binary = simple_thresh(red_channel,(140, 255))


        #thresholding on green channel
        green_channel = image[:,:,1]
        g_binary = simple_thresh(green_channel,(140, 255))

        #patitioning on r_binary
        r_binary_1 = r_binary[0:int(rows/3)-1,:]
        r_binary_2 = r_binary[int(rows/3):(2*int(rows/3))-1,:]
        r_binary_3 = r_binary[(2*int(rows/3)):rows-1,:]

        #patitioning on g_binary
        g_binary_1 = g_binary[0:int(rows/3)-1,:]
        g_binary_2 = g_binary[int(rows/3):(2*int(rows/3))-1,:]
        g_binary_3 = g_binary[(2*int(rows/3)):rows-1,:]

        #region sum
        r_binary_1_sum = r_binary_1.sum()
        r_binary_2_sum = r_binary_2.sum()
        r_binary_3_sum = r_binary_3.sum()

        #region sum
        g_binary_1_sum = g_binary_1.sum()
        g_binary_2_sum = g_binary_2.sum()
        g_binary_3_sum = g_binary_3.sum()

        if r_binary_1_sum > 2*r_binary_2_sum and r_binary_1_sum > 2*r_binary_3_sum:
            return TrafficLight.RED
        elif g_binary_3_sum > 2*g_binary_2_sum and g_binary_3_sum > 2*g_binary_1_sum:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN