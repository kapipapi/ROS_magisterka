import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO
from geometry_msgs.msg import PoseArray, Pose


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    super().__init__('image_subscriber')
    
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    
    self.publisher = self.create_publisher(PoseArray, 'object_positions', 10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
    # Load YOLOv8n model
    self.model = YOLO('/home/kacper/Downloads/yolov8n.pt')

  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    results = self.model.predict(source=current_frame, save=False)

    msg = PoseArray()
    for b in results[0].boxes:
      pose = Pose()
      pose.position.x = b.xywh[0,0].tolist()
      pose.position.y = b.xywh[0,1].tolist()

      msg.poses = msg.poses + [pose]

    self.publisher.publish(msg)

    # Display image
    # predicted = results[0].plot()
    # cv2.imshow("camera", predicted)
    # cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  
  image_subscriber = ImageSubscriber()
  
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()