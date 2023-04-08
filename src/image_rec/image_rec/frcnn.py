import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from std_msgs.msg import Header
import numpy as np

import torchvision
import torch
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

NUM_CLASSES = 4
RESIZE_TO = 512


class FrcnnNode(Node):

    def __init__(self):
        super().__init__('frcnn')
        self.get_logger().info("FrcnnNode node has been started")
        self.pub = self.create_publisher(BoundingBox2DArray, 'image_rec/frcnn_prediction', 10)
        self.sub = self.create_subscription(Image, 'image_rec/pre_processed_image', self.callback, 10)
        self.bridge = CvBridge()

        # set the computation device
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        # load the model and the trained weights
        self.model = create_model(num_classes=NUM_CLASSES).to(device)
        self.model.load_state_dict(torch.load(
            'src/image_rec/models/model13.pth', map_location=device
        ))
        self.model.eval()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = process_image(image)
        preds = self.model(image)

        weed_preds = []

        for label, box in zip(preds[0]['labels'], preds[0]['boxes']):
            if label == 'weed':
                weed_preds.append(box.detach().numpy().tolist())

        #convert weed bounding boxes to ROS message
        weed_preds = [convert_edgebox_to_cbox(bbox) for bbox in weed_preds]
        weed_preds_msg = bbox_list_to_msg(weed_preds)

        #update header information (propogate image header through)
        weed_preds_msg.header = data.header

        #publish
        self.pub.publish(weed_preds_msg)


def process_image(image):
    """Pre-processes image for FRCNN"""
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (RESIZE_TO, RESIZE_TO))
    image = image.transpose(2, 0, 1).astype(np.float32)
    image = image / 255.0
    image = torch.from_numpy(image)
    image = image.float()
    image = image.unsqueeze(0)
    return image

def convert_edgebox_to_cbox(edgebox):
    """Converts bounding box from [xmin, ymin, xmax, ymax] format 
        to [xcenter, ycenter, width, height]"""
    cx = edgebox[0] + edgebox[2]/2
    cy = edgebox[1] + edgebox[3]/2
    width = edgebox[2]-edgebox[0]
    height = edgebox[3]-edgebox[1]
    return [cx, cy, width, height]

def bbox_list_to_msg(bbox_list):
    """Converts list of bounding boxes to ROS message.
        Must be in [xcenter, ycenter, width, height] format"""
    bbox_arr_msg = BoundingBox2DArray()
    for bbox in bbox_list:
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = bbox[0]
        bbox_msg.center.y = bbox[1]
        bbox_msg.size_x = float(bbox[2])
        bbox_msg.size_y = float(bbox[3])

        bbox_arr_msg.boxes.append(bbox_msg)

    return bbox_arr_msg


def create_model(num_classes):
    
    # load Faster RCNN pre-trained model
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=False)
    
    # get the number of input features 
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # define a new head for the detector with required number of classes
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes) 

    """
    for param in model.parameters():
        param.requires_grad = False
        try:
            num_ftrs = model.fc.in_features
        except:
            num_ftrs = model.classifier[0].in_features
        # Here the size of each output sample is set to 2.
        # Alternatively, it can be generalized to nn.Linear(num_ftrs, len(class_names)).
        model.fc = nn.Linear(num_ftrs, NUM_CLASSES)
        model= model.to(device)
    """

    return model

def main(args=None):
    rclpy.init(args=args)

    frcnn_node = FrcnnNode()

    rclpy.spin(frcnn_node)

    frcnn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()