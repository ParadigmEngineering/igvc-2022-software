""" seg_to_bev.py

Generate BEV from semantically segmented front camera.
"""

import cv_bridge
import os
import numpy as np
import rospy
import tensorflow as tf
import yaml

from cv2 import cvtColor, COLOR_BGR2RGB
from sensor_msgs.msg import Image
from Cam2BEV.model import utils


class Model():    
    def __init__(
        self, 
        cam2bev_path,
        model_path,
        gpu_id=0,
        train_config_path="model/config.zedcam_F.unetxst.yml",
        oh_input_path="model/one_hot_conversion/convert_10_carla.xml",
        oh_label_path="model/one_hot_conversion/convert_3+occl_carla.xml",
        arch_path="model/architecture/uNetXST.py",
        homo_path="preprocessing/homography_converter/uNetXST_homographies/zedcam_F.py",
        topic_out="images/bev"
    ):
        self.cam2bev_path = cam2bev_path
        self.device = f"/device:GPU:{gpu_id}"
        self.bridge = cv_bridge.CvBridge() 
        self.publisher = rospy.Publisher(topic_out, Image, queue_size=10)

        with open(self._cfg_root(train_config_path)) as stream:
            train_config = yaml.safe_load(stream)

        self.one_hot_palette_input = utils.parse_convert_xml(self._cfg_root(oh_input_path))
        self.one_hot_palette_label = utils.parse_convert_xml(self._cfg_root(oh_label_path))
        
        n_classes_input = len(self.one_hot_palette_input)
        n_classes_label = len(self.one_hot_palette_label)
        n_inputs =  len(train_config['input-testing'])

        arch = utils.load_module(self._cfg_root(arch_path))
        uNetXSTHomographies = utils.load_module(self._cfg_root(homo_path))
    
        with tf.device(self.device):
            self.model = arch.get_network(
                (train_config['image-shape'][0], train_config['image-shape'][1], n_classes_input),
                n_classes_label,
                n_inputs=n_inputs,
                thetas=uNetXSTHomographies.H)
            
            self.model.load_weights(model_path)

    def predict(self, img_msg: Image):
        """ Feed an image message to the seg model, publish the output """
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        with tf.device(self.device):
            encoded_img = utils.one_hot_encode_image_op(cv_image, self.one_hot_palette_input)
            encoded_input = np.expand_dims(encoded_img, axis=0)
            pred_img = self.model.predict(encoded_input).squeeze()
            pred_img = utils.one_hot_decode_image(pred_img, self.one_hot_palette_label)

        pred_img = cvtColor(pred_img, COLOR_BGR2RGB)
        bev_img_msg = self.bridge.cv2_to_imgmsg(pred_img, encoding="passthrough")
        self.publisher.publish(bev_img_msg)

    def _cfg_root(self, postfix: str):
        return os.path.join(self.cam2bev_path, postfix)
 

def main():
    rospy.init_node("seg_to_bev", log_level=rospy.DEBUG)

    gpu_id = rospy.get_param("~gpu_id", 0)
    in_topic = rospy.get_param("~seg_img_in_topic", "images/seg")
    cam2bev_path = rospy.get_param("~cam2bev_path")
    model_folder_path = rospy.get_param("model_folder_path")
    model_name = rospy.get_param("~model_name")

    model = Model(cam2bev_path, os.path.join(model_folder_path, model_name), gpu_id)

    rospy.loginfo(f"Loading BEV ML model to GPU {gpu_id}...")
    rospy.loginfo("Waiting for segmented images...")
    rospy.Subscriber(in_topic, Image, model.predict)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
