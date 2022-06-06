""" zed_to_seg.py

Listen for zed images and segformer.
"""

import cv_bridge
import os
import numpy as np
import rospy

from cv2 import cvtColor, COLOR_BGR2RGB
from sensor_msgs.msg import Image
from torch import int8, nn
from transformers import SegformerForSemanticSegmentation, SegformerFeatureExtractor


class Model():
    palette = np.array([
        [ 81,   0,  81], 
        [250, 170,  30],
        [  0,   0,   0],
        [150, 120,  90],
        [255,   0,   0],
        [230, 150, 140],
        [ 70,  70,  70]])
    
    def __init__(self, model_path, gpu_id=0, topic_out="images/seg"):
        self.feature_extractor = SegformerFeatureExtractor(reduce_labels=True)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_path)
        self.publisher = rospy.Publisher(topic_out, Image, queue_size=10)
        self.device = f"cuda:{gpu_id}"
        self.bridge = cv_bridge.CvBridge() 
        self.model.to(self.device)
        
    def predict(self, img_msg: Image):
        """ Feed an image message to the seg model, publish the output """
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        encoding = self.feature_extractor(cv_image, return_tensors="pt")
        pixel_values = encoding.pixel_values.to(self.device)
        outputs = self.model(pixel_values=pixel_values)
        logits = outputs.logits.cpu()
        upsampled_logits = nn.functional.interpolate(logits,
                        size=(256, 512),
                        mode='bilinear',
                        align_corners=False)
        pred_img_encoded = upsampled_logits.argmax(dim=1)[0]

        pred_img = np.zeros((256, 512, 3), dtype=np.uint8)

        for label, color in enumerate(self.palette):
            pred_img[pred_img_encoded == label, :] = color
        
        pred_img = cvtColor(pred_img, COLOR_BGR2RGB)
        segmented_img_msg = self.bridge.cv2_to_imgmsg(pred_img, encoding="passthrough")
        self.publisher.publish(segmented_img_msg)

        
def main():
    rospy.init_node("zed_to_seg", log_level=rospy.DEBUG)
    
    # Load model
    gpu_id = rospy.get_param("~gpu_id")
    seg_image_pub_topic = rospy.get_param("~seg_image_out_topic")
    zed_topic = rospy.get_param("~zed_rgb_rectified_topic")
    model_folder_path = rospy.get_param("model_folder_path")
    model_name = rospy.get_param("~model_name")

    rospy.loginfo(f"Loading seg ML model to GPU {gpu_id}...")
    model = Model(os.path.join(model_folder_path, model_name), gpu_id, seg_image_pub_topic)

    rospy.loginfo("Waiting for zed images...")
    rospy.Subscriber(zed_topic, Image, model.predict)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
