import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from agrobot_msgs.msg import FrcnnBoundingBox
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Header
import numpy as np

import torch
import onnxruntime as ort

NUM_CLASSES = 4
RESIZE_TO = 512


class FrcnnPlantPredictionNode(Node):

    def __init__(self):
        super().__init__('frcnn')
        self.get_logger().info("Frcnn Plant Prediction Node node has been started")
        self.pub = self.create_publisher(
            FrcnnBoundingBox, 'image_rec/frcnn_plant_prediction', 10)
        self.sub = self.create_subscription(
            Image, 'image_rec/pre_processed_image', self.callback, 10)
        self.bridge = CvBridge()

        self.ort_sess = ort.InferenceSession('src/image_rec/models/frcnn.onnx')
        self.input_name = self.ort_sess.get_inputs()[0].name
        self.bbox_name = self.ort_sess.get_outputs()[0].name
        self.label_name = self.ort_sess.get_outputs()[1].name
        self.confidence_name = self.ort_sess.get_outputs()[2].name

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = process_image(image)
        out = self.ort_sess.run(
            [self.bbox_name, self.label_name, self.confidence_name], {self.input_name: image})
        box_pred = out[0]
        labels = out[1]
        confidence = out[2]

        weed_preds = []

        for box, label, conf in zip(box_pred, labels, confidence):
            # bean label is 0
            # maize label is 1
            # weed label is 2
            if label == 0:
                weed_preds.append([box.tolist(), conf, 'bean'])
            if label == 1:
                weed_preds.append([box.tolist(), conf, 'maize'])

        self.get_logger().info(str(weed_preds))

        # convert weed bounding boxes to ROS message
        weed_preds = [convert_edgebox_to_cbox(bbox, conf, plant_id) for bbox, conf, plant_id in weed_preds]
        weed_preds_msg = bbox_list_to_msg(weed_preds)

        # update header information (propogate image header through)
        weed_preds_msg.header = data.header

        # publish
        self.pub.publish(weed_preds_msg)


def process_image(image):
    """Pre-processes image for FRCNN"""
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (RESIZE_TO, RESIZE_TO))
    image = image.transpose(2, 0, 1).astype(np.float32)
    image = image / 255.0
    return np.array([image])


def convert_edgebox_to_cbox(edgebox, confidence, plant_id):
    """Converts bounding box from [xmin, ymin, xmax, ymax] format 
        to [xcenter, ycenter, width, height]"""
    cx = edgebox[0] + edgebox[2]/2
    cy = edgebox[1] + edgebox[3]/2
    width = edgebox[2]-edgebox[0]
    height = edgebox[3]-edgebox[1]
    return (cx, cy, width, height, confidence, plant_id)


def bbox_list_to_msg(bbox_list):
    """Converts list of bounding boxes to ROS message.
        Must be in [xcenter, ycenter, width, height] format"""
    bbox_arr_msg = FrcnnBoundingBox()
    for bbox in bbox_list:
        bbox_msg = BoundingBox2D()
        bbox_msg.center.x = bbox[0]
        bbox_msg.center.y = bbox[1]
        bbox_msg.size_x = float(bbox[2])
        bbox_msg.size_y = float(bbox[3])
        bbox_msg.confidence_val = float(bbox[4])
        bbox_msg.plant_id = int(bbox[5])
        bbox_arr_msg.boxes.append(bbox_msg)

    return bbox_arr_msg


def create_model(num_classes):

    # load Faster RCNN pre-trained model
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(
        pretrained=False)

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
