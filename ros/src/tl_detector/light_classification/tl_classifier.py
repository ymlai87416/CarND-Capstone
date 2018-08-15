from styx_msgs.msg import TrafficLight
from yolo3.yolo import YOLO
import numpy as np
from PIL import Image
from ssd_inception.model import Model
import cv2

class TLClassifier(object):
    def __init__(self, is_site, configuration):
        #TODO load classifier
        self.is_site = is_site
        self.site_use_yolo = False

        if is_site:

            if self.site_use_yolo:
                self.model = YOLO(model_path=configuration["model_path"], anchors_path=configuration["model_anchor"],
                                  classes_path=configuration["model_classes"])
                self.traffic_light_classes = 9  # TODO: hard code
            else:
                self.model = Model(model_path=configuration["model_path"])

        else:
            self.model = YOLO(model_path=configuration["model_path"], anchors_path=configuration["model_anchor"],
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

                if self.site_use_yolo:
                    pil_image = Image.fromarray(image_data)
                    out_boxes, out_scores, out_classes = self.model.detect_image(pil_image)

                    detected_states = []

                    for i, c in reversed(list(enumerate(out_classes))):
                        # predicted_class = self.class_names[c]
                        box = out_boxes[i]
                        score = out_scores[i]

                        if c == self.traffic_light_classes:
                            top, left, bottom, right = box
                            top = max(0, np.floor(top + 0.5).astype('int32'))
                            left = max(0, np.floor(left + 0.5).astype('int32'))
                            bottom = min(pil_image.size[1], np.floor(bottom + 0.5).astype('int32'))
                            right = min(pil_image.size[0], np.floor(right + 0.5).astype('int32'))
                            corp_traffic_light = image_data[top:bottom, left:right, :]
                            detected_states.append(self.detect_traffic_light_state_luminous(corp_traffic_light))

                    if len(detected_states) > 0:
                        (_, idx, counts) = np.unique(detected_states, return_index=True, return_counts=True)
                        index = idx[np.argmax(counts)]

                        return detected_states[index]
                    else:
                        return TrafficLight.UNKNOWN

                else:
                    return self.model.detect_traffic_light(image)

            else:
                pil_image = Image.fromarray(image_data)
                out_boxes, out_scores, out_classes = self.model.detect_image(pil_image)

                detected_states = []

                for i, c in reversed(list(enumerate(out_classes))):
                    #predicted_class = self.class_names[c]
                    box = out_boxes[i]
                    score = out_scores[i]

                    if c == self.traffic_light_classes:
                        top, left, bottom, right = box
                        top = max(0, np.floor(top + 0.5).astype('int32'))
                        left = max(0, np.floor(left + 0.5).astype('int32'))
                        bottom = min(pil_image.size[1], np.floor(bottom + 0.5).astype('int32'))
                        right = min(pil_image.size[0], np.floor(right + 0.5).astype('int32'))
                        corp_traffic_light = image_data[top:bottom, left:right, :]
                        detected_states.append(self.detect_traffic_light_state_color(corp_traffic_light))

                if len(detected_states) > 0:
                    (_, idx, counts) = np.unique(detected_states, return_index=True, return_counts=True)
                    index = idx[np.argmax(counts)]

                    return detected_states[index]
                else:
                    return TrafficLight.UNKNOWN

        return TrafficLight.UNKNOWN

    def detect_traffic_light_state_color(self, image):
        def simple_thresh(img, thresh=(0, 255)):
            # apply only thresholding on image plane and return binary image
            binary = np.zeros_like(img)
            binary[(img > thresh[0]) & (img <= thresh[1])] = 1
            return binary

        rows = image.shape[0]

        # thresholding on red channel
        red_channel = image[:, :, 0]

        # thresholding on green channel
        green_channel = image[:, :, 1]

        blue_channel = image[:, :, 2]

        # patitioning on r_binary
        r_binary = simple_thresh(red_channel, (196, 255))
        # plt.figure()
        # plt.imshow(r_binary)
        r_binary_1 = r_binary[0:int(rows / 3) - 1, :]
        r_binary_2 = r_binary[int(rows / 3):(2 * int(rows / 3)) - 1, :]
        r_binary_3 = r_binary[(2 * int(rows / 3)):rows - 1, :]
        is_red = (r_binary_1.sum() > 2 * (r_binary_2.sum())) & (r_binary_1.sum() > 2 * (r_binary_3.sum()))

        # (GREEN) patitioning on g_binary
        g_binary = simple_thresh(green_channel, (183, 255))
        # plt.figure()
        # plt.imshow(g_binary)
        g_binary_1 = g_binary[0:int(rows / 3) - 1, :]
        g_binary_2 = g_binary[int(rows / 3):(2 * int(rows / 3)) - 1, :]
        g_binary_3 = g_binary[(2 * int(rows / 3)):rows - 1, :]

        is_green = (g_binary_3.sum() > 2 * (g_binary_2.sum())) & (g_binary_3.sum() > 2 * (g_binary_1.sum()))

        # (YELLOW) partition on g_binary and b_binary
        g_binary = simple_thresh(green_channel, (146, 255))
        b_binary = simple_thresh(blue_channel, (143, 255))
        combine = g_binary + b_binary

        y_binary_1 = combine[0:int(rows / 3) - 1, :]
        y_binary_2 = combine[int(rows / 3):(2 * int(rows / 3)) - 1, :]
        y_binary_3 = combine[(2 * int(rows / 3)):rows - 1, :]

        is_yellow = (y_binary_2.sum() > 2 * (y_binary_1.sum())) & (y_binary_2.sum() > 2 * (y_binary_3.sum()))

        if is_red:
            return TrafficLight.RED
        elif is_yellow:
            return TrafficLight.YELLOW
        elif is_green:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN

    def detect_traffic_light_state_luminous(self, image):

        def adjust_histo(image):
            img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
            if np.max(img_yuv[:, :, 0]) - np.min(img_yuv[:, :, 0]) < 180:
                img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])
            img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
            return img_output

        image = adjust_histo(image)

        img_h, img_w, img_ch = image.shape
        l_channel = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)[:, :, 2]

        top_third_marker = int(img_h / 3)
        bottom_third_marker = img_h - top_third_marker

        # Magnitude of L is established for each section of the image
        top = 0
        mid = 0
        bottom = 0

        count_result = {'RED': 0, 'YELLOW': 0, 'GREEN': 0}

        for i in range(top_third_marker):
            for j in range(img_w):
                top += l_channel[i][j]
        count_result['RED'] = top  # compensate for R, G, B => L, U, V

        for i in range(top_third_marker, bottom_third_marker):
            for j in range(img_w):
                mid += l_channel[i][j]
        count_result['YELLOW'] = mid

        for i in range(bottom_third_marker, img_h):
            for j in range(img_w):
                bottom += l_channel[i][j]
        count_result['GREEN'] = bottom

        # The result is classified into one of the 3 colors and returned
        max_count = max(count_result, key=count_result.get)
        std = np.std(list(count_result.values()))
        mean = np.mean(list(count_result.values()))
        # print('x', std, std/mean)

        if (std / mean < 0.1):
            return TrafficLight.UNKNOWN
        if max_count == 'RED':
            return TrafficLight.RED
        elif max_count == 'YELLOW':
            return TrafficLight.YELLOW
        elif max_count == 'GREEN':
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN