import rclpy
import cv2
from datetime import datetime
import torch
import torchvision
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        print(f"torch: {torch.__version__} from {torch.__path__}, torchvision: {torchvision.__version__} from {torchvision.__path__}")
        # subscribe to the topic of processed images
        self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.detect_cones, 10)

        # Publish bounding boxes
        self.publisher_bboxes_ = self.create_publisher(Float32MultiArray, '/bounding_boxes', 10)

        self.model = torch.load('./src/perception/models/yolov5.pt', map_location=torch.device('cuda' if torch.cuda.is_available() else 'cpu'))

    def detect_cones(self, msg):
        # display the image data
        bridge = CvBridge()
        bgr_img = bridge.compressed_imgmsg_to_cv2(msg)

        # Convert BGR to RGB
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        detection = self.model(rgb_img)

        bboxes = detection.xyxy[0]

        # BGR colors: [blue, orange, yellow]
        cone_colors = [(255, 0, 0), (2, 139, 250), (0, 255, 255)]

        bboxes_msg = Float32MultiArray()

        bboxes_msg.layout.dim.append(MultiArrayDimension())
        bboxes_msg.layout.dim.append(MultiArrayDimension())
        bboxes_msg.layout.dim[0].label = 'Bounding Boxes'
        bboxes_msg.layout.dim[0].size = len(bboxes)
        bboxes_msg.layout.dim[1].label = 'Coordinates'
        bboxes_msg.layout.dim[1].size = 6

        bboxes_msg.data = torch.flatten(bboxes).tolist()

        self.publisher_bboxes_.publish(bboxes_msg)
        print(f"Published {len(bboxes)} bounding boxes at {str(datetime.now())}")

        for bbox in bboxes:
            cv2.rectangle(bgr_img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), cone_colors[int(bbox[5])], thickness=2)

        cv2.imshow('Detected cones', bgr_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDetectionNode())

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
