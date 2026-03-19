import sys
import os
import torch
import torchvision.transforms as transforms
import numpy as np
import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), 'models'))
from model_128_hinge import HingeClassifier
from model_hinge_angle import HingeAngleModel

BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))

def load_model(model_class, weights_path, device):
    model = model_class()
    model.to(device)
    model.load_state_dict(torch.load(weights_path)['model_state_dict'])
    model.eval()
    return model

class HingeModel():
    def __init__(self):
        # Load models
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.MCmodel = load_model(HingeClassifier, BASE_DIR + "/sam_ws/src/sam_ros/weight/hinge_mc_weights.pth", self.device)
        self.Anglemodel = load_model(HingeAngleModel, BASE_DIR + "/sam_ws/src/sam_ros/weight/hinge_angle_weights.pth", self.device)

    def preprocess_image(self, image, mask, target_size=(128, 128)):

        x, y, w, h = cv2.boundingRect(mask.astype(np.uint8))
        extracted_region = image[y:y+h, x:x+w]
        
        masked_region = np.zeros_like(extracted_region)
        masked_region[mask[y:y+h, x:x+w]] = extracted_region[mask[y:y+h, x:x+w]]
        
        resized_image = cv2.resize(masked_region, target_size)
        image_tensor = transforms.ToTensor()(resized_image).unsqueeze(0)

        image_tensor = image_tensor.to(self.device)

        index = self.MCmodel.forward(image_tensor).argmax().item()

        return index, masked_region

    def predict_angle(self, image, max_dim=128):
        image = cv2.resize(image, None, fx=0.5, fy=0.5)
        original_height, original_width = image.shape[:2]
        padded_image = np.zeros((max_dim, max_dim, 3), dtype=np.uint8)

        x_offset = (max_dim - original_width) // 2
        y_offset = (max_dim - original_height) // 2

        print(f"image shape: {image.shape}")
        print(f"padded_image shape: {padded_image.shape}")
        print(f"y_offset: {y_offset}, x_offset: {x_offset}")
        print(f"original_height: {original_height}, original_width: {original_width}")

        padded_image[y_offset:y_offset + original_height, x_offset:x_offset + original_width] = image
        image_tensor = transforms.ToTensor()(padded_image)

        with torch.no_grad():
            output = self.Anglemodel(image_tensor.to("cuda:0").unsqueeze(0))
            _, predicted_angle = torch.max(output, 1)

        return float(predicted_angle.item())
