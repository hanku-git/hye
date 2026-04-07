import sys
import os
import cv2
import torch
import numpy as np

from torchvision.transforms import ToTensor
from sklearn.decomposition import PCA

sys.path.append(os.path.join(os.path.dirname(__file__), 'models'))
from model_128_bolt import BoltClassifier

class BoltModel():
    def __init__(self):
        self.BCmodel = BoltClassifier()
        self.BCmodel.load_state_dict(torch.load("/home/korasrobotics/sam_ws/src/sam_ros/weight/240808_bolt_1000.pth")['model_state_dict'])
        self.BCmodel.eval()

    def extract_rgb_masks(self, image, masks):
        rgb_region = []
        rgb_resized_region = []
        for mask in masks:
            x, y, w, h = cv2.boundingRect(mask.astype(np.uint8))
            extracted_region = image[y:y+h, x:x+w]
            
            # Apply mask to the extracted region
            masked_region = np.zeros_like(extracted_region)
            masked_region[mask[y:y+h, x:x+w]] = extracted_region[mask[y:y+h, x:x+w]]
            
            resized_region = cv2.resize(masked_region, [128,128]) # resized for BCmodel
            rgb_region.append(masked_region)
            rgb_resized_region.append(resized_region)
        
        return rgb_resized_region

    def get_index(self, rgb_resized_region):
        index_ = [] # bolt
        tf = ToTensor()
        for image in rgb_resized_region:
            image_tensor = tf(image)
            index = self.BCmodel.forward(image_tensor.unsqueeze(0)).argmax().item()
            index_.append(index)

        return index_

    def find_major_minor_axes(self, image):
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image = image.astype(np.uint8)
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        contour_points = largest_contour[:, 0, :]
        pca = PCA(n_components=2)
        pca.fit(contour_points)
        # mean = np.mean(contour_points, axis=0)
        components = pca.components_
        explained_variance = pca.explained_variance_
        major_axis = components[0] * 2 * np.sqrt(explained_variance[0])
        minor_axis = components[1] * 2 * np.sqrt(explained_variance[1])
        return major_axis, minor_axis

    def segment_bolt_head_tail(self, image, major_axis):
        height, width = image.shape[:2]
        image_center = np.array([width / 2, height / 2])
        
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image = image.astype(np.uint8)
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        contour_points = largest_contour[:, 0, :]
        
        # Get a perpendicular vector to the major axis
        minor_axis = np.array([-major_axis[1], major_axis[0]])
        
        # Project points onto the minor axis
        projected = np.dot(contour_points - image_center, minor_axis)
        
        # Split points into upper and lower parts
        upper_points = contour_points[projected >= 0]
        lower_points = contour_points[projected < 0]
        
        # Count the number of pixels in each part
        upper_mask = np.zeros(image.shape, dtype=np.uint8)
        lower_mask = np.zeros(image.shape, dtype=np.uint8)
        cv2.drawContours(upper_mask, [upper_points], -1, 255, thickness=cv2.FILLED)
        cv2.drawContours(lower_mask, [lower_points], -1, 255, thickness=cv2.FILLED)
        
        upper_pixel_count = cv2.countNonZero(upper_mask)
        lower_pixel_count = cv2.countNonZero(lower_mask)
        
        return upper_points if upper_pixel_count > lower_pixel_count else lower_points, lower_points if upper_pixel_count > lower_pixel_count else upper_points

    def find_centroid(self, points):
        """Calculate the centroid of a set of points."""
        centroid = np.mean(points, axis=0)
        return centroid

    def find_point_on_segment(self, head_centroid, tail_centroid, ratio):
        """Find a point on the line segment connecting head_centroid and tail_centroid at the given ratio."""
        point = head_centroid + ratio * (tail_centroid - head_centroid)
        return point

    def calculate_angle_from_origin_to_vector(self, head_centroid, tail_centroid):
        """Calculate the angle from the origin to the vector from head to tail centroid."""
        vector = tail_centroid - head_centroid
        angle = - np.degrees(np.arctan2(vector[1], vector[0]))  # Angle relative to x-axis
        if angle < 0:
            angle = 360 + angle
        return angle