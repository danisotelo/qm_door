#!/usr/bin/env python3
import rospy
import rospkg
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

# Initialize the CvBridge
bridge = CvBridge()

# Function to load the model with custom weights
def load_model():
    # Get the path to the package
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('qm_vision')
    model_path = os.path.join(package_path, 'models', 'handle_detection.pt')

    # Load the model using the specified weights
    model = YOLO(model_path)
    return model

# Initialize the model
model = load_model()

def image_callback(data, image_pub):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        results = model.predict(cv_image, max_det = 1)
        annotated_image = results[0].plot()
        image_pub.publish(bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    except Exception as e:
        rospy.logerr("General error during image processing: {0}".format(e))

def main():
    rospy.init_node('handle_detection', anonymous = True)
    image_pub = rospy.Publisher('/yolo/annotated_image', Image, queue_size = 10)
    image_sub = rospy.Subscriber('camera/color/image_raw', Image, image_callback, callback_args=image_pub)
    rospy.spin()

if __name__ == '__main__':
    main()