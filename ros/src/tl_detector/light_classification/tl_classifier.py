from styx_msgs.msg import TrafficLight
import rospy
import cv2
import numpy as np
import yaml
import sys
from PIL import Image
import tensorflow as tf
from object_detection.utils import label_map_util

NUM_CLASSES = 4

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        model_path = self.config['frozen_model_path']

        self.detection_graph = tf.Graph()
        self.load_frozen_model(model_path)
        self.categories, self.category_index = self.load_label_map('tl_label_map.pbtxt')
        # This setups the tensorflow session once for all predctions
        self.setup_sess()

    def load_frozen_model(self, model_path):
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def load_label_map(self, label_map_path):
        label_map = label_map_util.load_labelmap(label_map_path)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
        return categories, category_index

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    def get_category(self, index):
        for category in self.categories:
            if category['id'] == index:
                return category
        return None


    def setup_sess(self):
        with self.detection_graph.as_default():
            self.sess = tf.Session(graph=self.detection_graph)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


    def predict(self, image):
        # init to unknown
        index = 4
        image_np = self.load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        # awoo:  Hopefully, this will remove setting up tensorflow session for each predction.
        #        The session is now being setup by setup_sess() called by init().
        #        It's unclear how much speed up we get here.
        #
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        top_score = scores[0][0]
        if top_score < 0.5:
            print("{},{}".format("Unknown", top_score))
        else:
            class_index = int(classes[0][0])
            category = self.get_category(class_index)
            if category is not None:
                # label index is 1 higher than TrafficLight definition.
                # so, we need to substract 1
                index = class_index-1
                #print("{},{},{}".format(category['name'], top_score, index))

        """
        #
        # The old way of creating a session everytime we make a predction
        #
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})

                top_score = scores[0][0]
                if top_score < 0.5:
                    print("1.{},{}".format("Unknown", top_score))
                else:
                    class_index = int(classes[0][0])
                    category = self.get_category(class_index)
                    if category is not None:
                        # label index is 1 higher than TrafficLight definition.
                        # so, we need to substract 1
                        index = class_index
                        print("2.{},{},{}".format(category['name'], top_score, index))
        """
        return index

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        start = rospy.get_rostime()

        cv2_im = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        # PIL use RGB
        pil_im = Image.fromarray(cv2_im)
        # dump to jpg if we want to see the image
        #pil_im.save('test.jpg')
        index = self.predict(pil_im)

        duration = rospy.get_rostime()-start
        #print("Duration %i %i", duration.secs, duration.nsecs)

        #return TrafficLight.UNKNOWN
        return index
