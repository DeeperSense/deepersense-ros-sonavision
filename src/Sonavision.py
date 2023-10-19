#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from ros1_sonavision_inference.cfg import SonavisionConfig
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pdb
import tensorflow as tf
import numpy as np


class Sonavision:
    def __init__(
        self,
        # threshold=0.3,
        # camera_topic="/usb_cam/image_raw",
        # sonar_topic="/drawn_sonar_rect",
        # model_path="../tf/models/",
        # checkpoint_path="../tf/checkpoints/ckpt-23",
    ):
        rospy.init_node("ros1_sonavision_inference")
        rospy.loginfo("Starting Sonavision as ros1_sonavision_inference.")

        self.dynamic_reconfigure_server = Server(
            SonavisionConfig, self.reconfigure_callback
        )

        # get params
        self.threshold = rospy.get_param("~threshold", 0.3)
        _camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        _sonar_topic = rospy.get_param("~sonar_topic", "/drawn_sonar_rect")
        self.model_path = rospy.get_param("~model_path", "../tf/models/")
        self.checkpoint_path = rospy.get_param(
            "~checkpoint_path", "../tf/checkpoints/ckpt-23"
        )

        self.frame_id_counter = 0
        self.blur_level = 0.0
        self.darkness_level = 0.0

        self.cv_bridge = CvBridge()
        self.raw_data_pub = rospy.Publisher("raw_data", Image, queue_size=10)
        self.inference_pub = rospy.Publisher("inference", Image, queue_size=10)

        self.camera_sub = Subscriber(_camera_topic, Image)
        self.sonar_sub = Subscriber(_sonar_topic, Image)

        # Load model
        rospy.loginfo("Loading model from " + self.model_path)
        self.model = tf.keras.models.load_model(self.model_path)
        rospy.loginfo("Model loaded.")

        # restore checkpoint
        rospy.loginfo("Restoring checkpoint from " + self.checkpoint_path)
        self.checkpoint = tf.train.Checkpoint(generator=self.model)
        self.checkpoint.restore(self.checkpoint_path).expect_partial()
        rospy.loginfo("Checkpoint restored.")

        self.ts = ApproximateTimeSynchronizer(
            [self.camera_sub, self.sonar_sub], queue_size=50, slop=self.threshold
        )
        self.ts.registerCallback(self.sync_callback)

    def normalize_inputs(self, x):
        """Normalize a list of 8-bit images to the range of [-1,1]"""
        normalized = []
        for img in x:
            normalized.append((img / 127.5) - 1)
        return normalized

    def blur(self, image, val):
        var = 10 * val  # variance
        k = int(np.ceil(100 * val) // 2 * 2 + 1)
        kernel = (k, k)  # kernel
        blurred = cv2.GaussianBlur(
            src=image, ksize=kernel, sigmaX=var, sigmaY=var
        )  # gaussian blur
        return blurred

    def darken(self, image, value):
        """Darken an image by a value between 0 and 1"""
        dark = (1 - value) * image
        return dark

    def reconfigure_callback(self, config, level):
        self.blur_level = config["blur_level"]
        self.darkness_level = config["darkness_level"]
        rospy.loginfo(
            """[Reconfigure Request]: Blur: {}, Darkness: {}""".format(
                self.blur_level, self.darkness_level
            ),
        )
        return config

    def sync_callback(self, camera_msg, sonar_msg):
        # Process data from both subscribers and combine them into a new  message MatchedSonarCameraImages
        # rospy.loginfo(
        #     "Received data from both subscribers. Processing and publishing combined data. frame_id: "
        #     + str(self.frame_id_counter)
        # )

        self.frame_id_counter += 1

        camera_cv_img = self.cv_bridge.imgmsg_to_cv2(
            camera_msg, desired_encoding="passthrough"
        )
        sonar_cv_img = self.cv_bridge.imgmsg_to_cv2(
            sonar_msg, desired_encoding="passthrough"
        )

        _, camera_w, _ = camera_cv_img.shape

        # crop camera image
        camera_cv_img_mono = camera_cv_img[:, 0 : camera_w // 2, :]
        camera_cv_img_mono_cropped = camera_cv_img_mono[104:616, 128:1152, :]

        # darken camera image
        camera_cv_img_mono_cropped_darkened = self.darken(
            camera_cv_img_mono_cropped, self.darkness_level
        )
        # blur camera image
        camera_cv_img_mono_cropped_darkened_blurred = self.blur(
            camera_cv_img_mono_cropped_darkened, self.blur_level
        )

        # crop sonar image
        sonar_h, _, _ = sonar_cv_img.shape
        if sonar_h % 2 == 0:
            pt, pb = (1024 - sonar_h) // 2, (1024 - sonar_h) // 2
        else:
            pt, pb = (1024 - sonar_h) // 2, (1024 - sonar_h) // 2 + 1

        sonar_cv_img_padded = cv2.copyMakeBorder(
            sonar_cv_img, pt, pb, 0, 0, cv2.BORDER_CONSTANT, None, 0
        )
        sonar_cv_img_padded_rotated = cv2.rotate(
            sonar_cv_img_padded, cv2.ROTATE_90_CLOCKWISE
        )

        assert (
            camera_cv_img_mono_cropped_darkened_blurred.shape
            == sonar_cv_img_padded_rotated.shape
        )

        camera_tf = tf.cast(camera_cv_img_mono_cropped_darkened_blurred, tf.float32)
        sonar_tf = tf.cast(sonar_cv_img_padded_rotated, tf.float32)

        camera_tf, sonar_tf = self.normalize_inputs([camera_tf, sonar_tf])

        prediction = self.model(
            [
                camera_tf[None, :, :, :],
                sonar_tf[None, :, :, :],
            ],
            training=True,
        )

        inferred_img = prediction[0].numpy()
        inferred_img = inferred_img * 0.5 + 0.5

        inferred_img = cv2.normalize(
            inferred_img, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U
        )

        # horizontally concat images
        out_img = cv2.hconcat(
            [
                camera_cv_img_mono_cropped_darkened_blurred.astype(np.uint8),
                sonar_cv_img_padded_rotated,
                inferred_img,
            ]
        )

        # show out_img
        # cv2.imshow("out_img", out_img)
        # cv2.waitKey(1)

        out_msg = self.cv_bridge.cv2_to_imgmsg(out_img, encoding="bgr8")
        self.inference_pub.publish(out_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Sonavision()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start Sonavision.")
