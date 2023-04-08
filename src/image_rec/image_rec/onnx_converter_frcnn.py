import torchvision
import torch
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor



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

if __name__ == '__main__':
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    # load the model and the trained weights
    model = create_model(num_classes=4).to(device)
    model.load_state_dict(torch.load(
        'src/image_rec/models/model13.pth', map_location=device
    ))
    model.eval()

    dummy_input = torch.randn(1, 3, 512, 512, requires_grad=True)

    # Export the model
    torch.onnx.export(model,               # model being run
                    dummy_input,                         # model input (or a tuple for multiple inputs)
                    "src/image_rec/models/frcnn.onnx",   # where to save the model (can be a file or file-like object)
                    export_params=True,        # store the trained parameter weights inside the model file
                    opset_version=11,          # the ONNX version to export the model to
                    do_constant_folding=True,  # whether to execute constant folding for optimization
                    input_names = ['images'],   # the model's input names
                    output_names = ['bboxes', 'labels', 'confidence'], # the model's output names
                    dynamic_axes={'images' : {0 : 'batch_size'},    # variable length axes
                                    'bboxes' : {0 : 'batch_size'},
                                    'labels' : {0 : 'batch_size'},
                                    'confidence' : {0 : 'batch_size'}})