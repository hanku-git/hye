#!/usr/bin/env python3

import numpy as np
import cv2
import sys
import time
import os
import torch
import threading
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

# sys.path.append("..")
# sys.path.append('/sam_util')

# print("--------------")
# print(sys.path)
# print("--------------")


from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# import visualize

# from sam_ros import coco
# from mask_rcnn_ros import model as modellib
from hanyang_matching_msgs.msg import Resultsam
from hanyang_matching_msgs.msg import BpUiCommandLearning
import time
import json

from torchvision import transforms
from model import MaskClassifier

BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))

IMGPATH = BASE_DIR + "/[zivid_scan_data]/"
sam_checkpoint = BASE_DIR + "/[sam_weight]/sam_vit_h_4b8939.pth"  # sam_vit_h_4b8939 sam_vit_b_01ec64
model_type = "vit_h"
device = "cuda"
test = False
# save_img = True
min_size_thre = 0.8 
max_size_thre = 6.0
# overlap_thre = 0.9
nearby_radius = 10

# Pixel min: (211, 375), max: (1140,1022)
# bin_x_min = 164
# bin_x_max = 1110
# bin_y_min = 375
# bin_y_max = 980
# bin_green
# bin_x_min = 0
# bin_x_max = 1440
# bin_y_min = 0
# bin_y_max = 1080


CLASS_NAMES = ['BG','peg']
###################################

### 2) 필요할 때만 memory 할당
# config = tf.ConfigProto()
# config.gpu_options.allow_growth = True
# sess = tf.Session(config=config)
######
"""
Using a SAM model, generates masks for the entire image.
Generates a grid of point prompts over the image, then filters
low quality and duplicate masks. The default settings are chosen
for SAM with a ViT-H backbone.

Arguments:
    model (Sam): The SAM model to use for mask prediction.
    points_per_side (int or None): The number of points to be sampled
    along one side of the image. The total number of points is
    points_per_side**2. If None, 'point_grids' must provide explicit
    point sampling.
    points_per_batch (int): Sets the number of points run simultaneously
    by the model. Higher numbers may be faster but use more GPU memory.
    pred_iou_thresh (float): A filtering threshold in [0,1], using the
    model's predicted mask quality.
    stability_score_thresh (float): A filtering threshold in [0,1], using
    the stability of the mask under changes to the cutoff used to binarize
    the model's mask predictions.
    stability_score_offset (float): The amount to shift the cutoff when
    calculated the stability score.
    box_nms_thresh (float): The box IoU cutoff used by non-maximal
    suppression to filter duplicate masks.
    crop_n_layers (int): If >0, mask prediction will be run again on
    crops of the image. Sets the number of layers to run, where each
    layer has 2**i_layer number of image crops.
    crop_nms_thresh (float): The box IoU cutoff used by non-maximal
    suppression to filter duplicate masks between different crops.
    crop_overlap_ratio (float): Sets the degree to which crops overlap.
    In the first crop layer, crops will overlap by this fraction of
    the image length. Later layers with more crops scale down this overlap.
    crop_n_points_downscale_factor (int): The number of points-per-side
    sampled in layer n is scaled down by crop_n_points_downscale_factor**n.
    point_grids (list(np.ndarray) or None): A list over explicit grids
    of points used for sampling, normalized to [0,1]. The nth grid in the
    list is used in the nth crop layer. Exclusive with points_per_side.
    min_mask_region_area (int): If >0, postprocessing will be applied
    to remove disconnected regions and holes in masks with area smaller
    than min_mask_region_area. Requires opencv.
    output_mode (str): The form masks are returned in. Can be 'binary_mask',
    'uncompressed_rle', or 'coco_rle'. 'coco_rle' requires pycocotools.
    For large resolutions, 'binary_mask' may consume large amounts of
    memory.
"""

def remove_glare(img):

    # convert to gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur( gray, (9,9), 0 )

    # threshold grayscale image to extract glare
    mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

    # Optionally add some morphology close and open, if desired
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # use mask with input to do inpainting
    result = cv2.inpaint(img, mask, 25, cv2.INPAINT_TELEA) 

    return result


class SamNode(object):
    def __init__(self):
        self.node = rclpy.create_node('sam_zivid')
        self._cv_bridge = CvBridge()
        self.init_set()
        self._publish_rate = self.node.declare_parameter('~publish_rate', 100).value

        #### SAM
        print("Loading model...")
        self._target_name = 'default'
        self.setSamParameters()
        if not self.no_detection_mode:
            self.sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
            self.sam.to(device=device)

        self._visualization = self.node.declare_parameter('~visualization', True)
        print("Loading finished!")

        if self.no_detection_mode:
            print("No AI Detection Mode!")
        else:
            print("AI Detection Mode!")

            #### Classifier
            model_path_str = "/home/" + username_ + "/sam_ws/src/sam_ros/weight/final_model_new.pth"
            model_path = self.node.declare_parameter('~model_path', model_path_str).value
            cuda_device = self.node.declare_parameter('~cuda_device', 'cuda:0').value
            self.classifier = MaskClassifier(5)
            self.classifier.load_state_dict(torch.load(model_path)['model_state_dict'])
            self.classifier.to(cuda_device)
            self.classifier.eval()

            #### Transform
            self.transform = transforms.Compose([
                transforms.ToTensor(),
            ])



        ####
        self.i = 0
        self._class_names = CLASS_NAMES
        self._cad_bounding_box = []

        self._last_msg = None
        self._msg_lock = threading.Lock()
        self._last_msg_2 = None
        self._msg_lock_2 = threading.Lock()

        self.debug_mode = False
        # self._class_colors = visualize.random_colors(8)

    def init_set(self):
        self.whole_size = 0
        self.size_set = []
        self.ratio_set = []
        self.overlap_pair = []        

    def setSamParameters(self):

        # with open (os.getcwd() + "/src/sam_ros/config/sam_config.json", "r") as f:
        with open ("/home/" + username_ + "/[scanDataHanyang]/sam_config/sam_config.json", "r") as f:
            self.sam_config = json.load(f)
            # Exact key only (no alias/fallback)
            conf = self.sam_config[self._target_name]
            self.no_detection_mode = conf['no_detection_mode']

            print("-------------------\n")
            print("--- sam_config ----\n")
            print("Target object: ", self._target_name)

            if self.no_detection_mode:
                print("No Detection Mode!")
                print("-------------------\n", f"(no_detection_pixel_min_h: {conf['no_detection_pixel_min_h']:d})\n", 
                                                f"(no_detection_pixel_min_v: {conf['no_detection_pixel_min_v']:d})\n",
                                                f"(no_detection_pixel_length_h: {conf['no_detection_pixel_length_h']:d})\n",
                                                f"(no_detection_pixel_length_v: {conf['no_detection_pixel_length_v']:d})\n",
                                                f"(no_detection_pixel_radius: {conf['no_detection_pixel_radius']:d})\n")
            else:
                print("AI Detection Mode!")
                print("-------------------\n", f"(points_per_side: {conf['points_per_side']:d})\n", 
                                                f"(points_per_batch: {conf['points_per_batch']:d})\n",
                                                f"(pred_iou_thresh: {conf['pred_iou_thresh']:.2f})\n",
                                                f"(stability_score_thresh: {conf['stability_score_thresh']:.2f})\n",
                                                f"(min_mask_region_area: {conf['min_mask_region_area']:d})\n",
                                                f"\n(min_area: {conf['min_area']:d})\n",
                                                f"\n(min_area: {conf['min_area']:d})\n",
                                                f"(max_area: {conf['max_area']:d})\n",
                                                f"(mean_area: {conf['mean_area']:d})\n")
            print("-------------------\n")
        ###################


        ###################
        if not self.no_detection_mode:
            output_mode = "binary_mask" # "coco_rle", "binary_mask", "uncompressed_rle"
            self.mask_generator = SamAutomaticMaskGenerator(
                model=self.sam,
                output_mode=output_mode,
                # points_per_side=18,
                # points_per_batch=16,
                # pred_iou_thresh=0.95,
                # stability_score_thresh=0.97,
                # stability_score_offset=1,

                # points_per_side=12,
                # points_per_batch=16,
                # pred_iou_thresh=0.9,
                # stability_score_thresh=0.75,
                # stability_score_offset=0.75,

                # points_per_side=12,
                # points_per_batch=16,
                # pred_iou_thresh=0.8,
                # stability_score_thresh=0.5,
                # stability_score_offset=0.5,

                points_per_side         = conf['points_per_side'],
                points_per_batch        = conf['points_per_batch'],
                pred_iou_thresh         = conf['pred_iou_thresh'],
                stability_score_thresh  = conf['stability_score_thresh'],
                stability_score_offset  = conf['stability_score_offset'],

                box_nms_thresh=0.7,
                crop_n_layers=0,
                crop_nms_thresh=0.7,
                crop_overlap_ratio=1500 / 1500,
                crop_n_points_downscale_factor=1,
                # min_mask_region_area=6000,  # Requires open-cv to run post-processing
                min_mask_region_area = conf['min_mask_region_area'],  # Requires open-cv to run post-processing
            )

        ####
        self._min_area = conf['min_area']
        self._max_area = conf['max_area']
        self._mean_area = conf['mean_area']
        self._is_mean_area_assigned = conf['is_mean_area_assigned']
        self._do_additional_processing = conf['do_additional_processing']
        self._do_classification = self.sam_config[self._target_name]['do_classification']

        self._overlap_threshold = conf['overlap_threshold']
        self._save_img = conf['save_img']

        self._do_min_max_size_filtering = conf['do_min_max_size_filtering']
        self._do_overlap_filtering = conf['do_overlap_filtering']
        
        self.bin_x_min = self.sam_config[self._target_name]['bin_area'][0]
        self.bin_x_max = self.sam_config[self._target_name]['bin_area'][1]
        self.bin_y_min = self.sam_config[self._target_name]['bin_area'][2]
        self.bin_y_max = self.sam_config[self._target_name]['bin_area'][3]
        self.debug_mode = self.sam_config[self._target_name]['debug_mode']


        # No detection mode
        self.no_detection_pixel_min_h = self.sam_config[self._target_name]['no_detection_pixel_min_h']
        self.no_detection_pixel_min_v = self.sam_config[self._target_name]['no_detection_pixel_min_v']
        self.no_detection_pixel_length_h = self.sam_config[self._target_name]['no_detection_pixel_length_h']
        self.no_detection_pixel_length_v = self.sam_config[self._target_name]['no_detection_pixel_length_v']
        self.no_detection_pixel_radius = self.sam_config[self._target_name]['no_detection_pixel_radius']
        self.no_detection_is_rectangular_box = self.sam_config[self._target_name]['no_detection_is_rectangular_box']
        self.no_detection_is_horizontal_cut_applied = self.sam_config[self._target_name]['no_detection_is_horizontal_cut_applied']
        self.no_detection_is_vertical_cut_applied = self.sam_config[self._target_name]['no_detection_is_vertical_cut_applied']


        if self.debug_mode:
            print("debug_mode!!!")

    def create_no_detection_mask(self, np_image):
        """
        detection_mode가 아닐 때 사용될 새로운 mask를 생성하는 함수.
        
        Args:
            self: 클래스 내부에서 정의된 변수들을 사용.
            np_image (numpy.ndarray): 입력 이미지 (H x W x C).

        Returns:
            list: 하나의 mask가 담긴 리스트
        """
        # 이미지 크기 가져오기
        img_rows, img_cols, _ = np_image.shape

        # 최대 row, col 값 업데이트
        no_detection_pixel_max_v = self.no_detection_pixel_min_v + self.no_detection_pixel_length_v
        no_detection_pixel_max_h = self.no_detection_pixel_min_h + self.no_detection_pixel_length_h

        # segmentation 초기화 (모든 픽셀 False)
        segmentation = np.zeros((img_rows, img_cols), dtype=bool)

        # 바운딩 박스 중심 계산
        center_row = (self.no_detection_pixel_min_v + no_detection_pixel_max_v) // 2
        center_col = (self.no_detection_pixel_min_h + no_detection_pixel_max_h) // 2

        if self.no_detection_is_rectangular_box:
            print("Rectangular Box")
            # 사각형 박스 설정
            segmentation[self.no_detection_pixel_min_v:no_detection_pixel_max_v,
                        self.no_detection_pixel_min_h:no_detection_pixel_max_h] = True
        else:
            print("Circular Box")
            # 원형 영역 설정 (반지름 조건만 고려)
            for r in range(max(0, center_row - self.no_detection_pixel_radius), 
                        min(img_rows, center_row + self.no_detection_pixel_radius)):
                for c in range(max(0, center_col - self.no_detection_pixel_radius), 
                            min(img_cols, center_col + self.no_detection_pixel_radius)):
                    if (r - center_row) ** 2 + (c - center_col) ** 2 <= self.no_detection_pixel_radius ** 2:
                        segmentation[r, c] = True

            # ✅ 추가적인 필터링: 수직 및 수평 컷 적용
            if self.no_detection_is_vertical_cut_applied:
                segmentation[:self.no_detection_pixel_min_v, :] = False  # 상단 영역 제거
                segmentation[no_detection_pixel_max_v:, :] = False  # 하단 영역 제거

            if self.no_detection_is_horizontal_cut_applied:
                segmentation[:, :self.no_detection_pixel_min_h] = False  # 좌측 영역 제거
                segmentation[:, no_detection_pixel_max_h:] = False  # 우측 영역 제거


        # 새로운 마스크 객체 생성
        new_mask = {
            'segmentation': segmentation,
            'bbox': [self.no_detection_pixel_min_h, self.no_detection_pixel_min_v, 
                    self.no_detection_pixel_length_h, self.no_detection_pixel_length_v],  # 길이 유지
            'area': np.sum(segmentation),  # True인 픽셀 개수
            'point_coords': [[center_col, center_row]],  # 중심 좌표
            'predicted_iou': 1.0291153192520142,
            'stability_score': 0.9844344854354858,
            'crop_box': [0, 0, img_cols, img_rows]  # 이미지 크기를 기반으로 crop_box 설정
        }

        # masks 리스트에 추가
        masks = [new_mask]

        return masks




    def run(self):
        self._result_pub = self.node.create_publisher(Resultsam, '/sam_zivid/result', 1)
        vis_pub = self.node.create_publisher(Image, '/sam_zivid/visualization', 1)
        sub = self.node.create_subscription(Image, '/zivid/color/image_color',
                                    self._image_callback, 1)
        weight_load_sub = self.node.create_subscription(BpUiCommandLearning, '/sam_zivid/load_weight',
                                    self._load_weight_callback, 1)

        rate = self.node.create_rate(self._publish_rate)

        while rclpy.ok():            
            rclpy.spin_once(self.node)
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue
            
            if msg is not None:
                print('[sam_node.py] image input!')
                start = time.time()
                np_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.init_set()
                # Run detection
                start_sam = time.time()                
                img_sam = np_image
                # img_sam = remove_glare(np_image)

                if self._save_img:                    
                    filename = IMGPATH + "%d.png" % self.i
                    cv2.imwrite(os.path.join("", filename), img_sam)

                if self.no_detection_mode: # No detection mode
                    masks = self.create_no_detection_mask(img_sam)
                    result_msg, result_masks = self._build_no_detection_mode_result_msg(msg, img_sam, masks)
                    end_sam = time.time()      
                    print(f"No detection processing time: {end_sam - start_sam:.5f} sec")    
                else: # SAM Mode
                    masks = self.mask_generator.generate(img_sam)
                    end_sam = time.time()      
                    result_msg, result_masks = self._build_result_msg(msg, img_sam, masks)
                    print(f"sam detection runtime: {end_sam - start_sam:.5f} sec")    
                

                ######################
                #### 241212, 현대차
                # filtered_msg = self._class_filter(result_msg)

                # for j in range(len(filtered_msg.masks)):
                #     if self._save_img:
                #         if not os.path.exists(IMGPATH+"/target_masks/%d" %self.i):
                #             os.makedirs(IMGPATH+"/target_masks/%d" %self.i)
                #         filename = IMGPATH+"/target_masks/%d/%d.png" %(self.i, j)
                #         mask2save = self._cv_bridge.imgmsg_to_cv2(filtered_msg.masks[j], 'bgr8')
                #         cv2.imwrite(os.path.join("", filename), mask2save)

                # self._result_pub.publish(filtered_msg)                

                # if result_msg.class_cnt == 0 :
                #     print('No mask detected!')
                # else :
                #     print('Detection!')
                #     print(f"Dectected Mask #: {len(result_msg.masks)}")
                #     print(f"published!! mask number: {len(filtered_msg.masks)}")       
                # 
                # 
                ######################

                ######################
                #### 241212, 드럼통
                self._result_pub.publish(result_msg)                

                if result_msg.class_cnt == 0 :
                    print('No mask detected!')
                else :
                    print('Detection!')
                    print(f"Dectected Mask #: {len(result_msg.masks)}")
                    print(f"published!! mask number: {len(result_msg.masks)}")                                 
                ######################


                # Visualize results
                if self._visualization:
                    vis_image = self._visualize(masks, np_image, result_msg, result_masks)
                    cv_result = np.zeros(shape=vis_image.shape, dtype=np.uint8)
                    cv2.convertScaleAbs(vis_image, cv_result)
                    
                    if self._save_img:                    
                        filename = IMGPATH + "sam_%d.jpg" % self.i
                        cv2.imwrite(os.path.join("", filename), cv_result)
                        self.i += 1

                    image_msg = self._cv_bridge.cv2_to_imgmsg(cv_result, 'bgra8')
                    vis_pub.publish(image_msg)
                
                end = time.time()
                print(f"runtime: {end - start:.5f} sec")
                print("sam_ready")
            
            #### Loading weight
            if self._msg_lock_2.acquire(False):
                msg = self._last_msg_2
                self._last_msg_2 = None
                self._msg_lock_2.release()
                
            else:
                rate.sleep()
                continue

            if msg is not None:
                self._class_names = ['BG', msg.target_name]
                # self._target_name = 'irl_classifier'
                self._target_name = msg.target_name
                self.setSamParameters()
                print('Parameter initialize! (object: ' + msg.target_name + ')')
                if self._is_mean_area_assigned:
                    print(f"Assigned mean size is {self._mean_area:d}")
                else:
                    print("Mean size should be assigned!")

            # print('--')
            # rate.sleep()

    def show_anns(self,anns):
        if len(anns) == 0:
            return
        sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)

        img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
        img[:,:,3] = 0
        for ann in sorted_anns:
            m = ann['segmentation']
            color_mask = np.concatenate([np.random.random(3), [0.4]])
            img[m] = color_mask

        return img

    def _build_result_msg(self, msg, img_sam, result):
        result_msg = Resultsam()
        result_msg.header = msg.header
        detected_class_cnt = 0
        print('_build_result_msg!!!')
        masks = []
        masks_area = []
        boxes = []

        if self._do_classification:
            img_rgb = cv2.resize(img_sam, (512,512))
            img_tensor = self.transform(img_rgb).to("cuda:0")

        if self.debug_mode:
            for i, mask_data in enumerate(result):
                print(f"(Raw data) mask_data[{i:d}]['area'] is {mask_data['area']:.3f}")

        for j, mask_data in enumerate(result):
            # mask_data["bbox"]: the boundary box detection in xywh format
            box = RegionOfInterest()
            box.x_offset = int(mask_data["bbox"][0]) # bbox_x0, 픽셀 좌표계 기준 min_x value
            box.y_offset = int(mask_data["bbox"][1]) # bbox_y0, 픽셀 좌표계 기준 min_y value
            box.width = int(mask_data["bbox"][2]) # bbox_w
            box.height = int(mask_data["bbox"][3]) # bbox_h

            # class_id = masks['class_ids'][i]        

            score = mask_data["predicted_iou"] # the model's own prediction for the quality of the mask
            # score = mask_data["stability_score"] # an additional measure of mask quality
            result_msg.scores.append(score)

            if self._save_img:
                if not os.path.exists(IMGPATH+"/raw_img/%d" %self.i):
                    os.makedirs(IMGPATH+"/raw_img/%d" %self.i)
                print(f"raw_img #{j:d}, raw_mask_sum is {mask_data['area']:.3f}")
                filename = IMGPATH+"/raw_img/%d/%d_%d.png" %(self.i, self.i, j)
                cv2.imwrite(os.path.join("", filename), mask_data["segmentation"] * 255)
                
            # bin_area
            # if (box.x_offset > self.bin_x_min and box.y_offset > self.bin_y_min and
            #     self.bin_x_max > box.x_offset+box.width and self.bin_y_max > box.y_offset+box.height and
            #     box.width<self.bin_x_max*0.7 and box.height<self.bin_y_max*0.7):
            print(f"#{j:d} x:{box.x_offset} y: {box.y_offset}")
            if (self.bin_x_min <= box.x_offset and 
                self.bin_y_min <= box.y_offset and
                self.bin_x_max >= box.x_offset + box.width and
                self.bin_y_max >= box.y_offset + box.height and
                box.width <= (self.bin_x_max - self.bin_x_min) and
                box.height <= (self.bin_y_max - self.bin_y_min)):

                print(f"#{j:d} Box size in range!")

                if self._do_min_max_size_filtering:
                    # Specify the minimum and maximum area value you want to filter by
                    # 최소 면적보다 작거나 평균의 2.5배보다 큰 면적을 갖는 마스크는 제외하도록 처리
                    # if mask_data['area'] > self._min_area and mask_data['area'] < 3*self._mean_area:
                    # if mask_data['area'] > self._min_area and mask_data['area'] < 1.5*self._mean_area:
                    
                    # [240214 변경] 최소 면적보다 작거나 최대 면적보다 큰 면적을 갖는 마스크는 제외하도록 처리
                    if mask_data['area'] > self._min_area and mask_data['area'] < self._max_area:
                        if self.debug_mode:
                            print(f"(in range)mask_data['area'] is {mask_data['area']:.3f}")
                        masks.append(mask_data['segmentation'])
                        masks_area.append(mask_data['area'])
                        boxes.append(box)
                        detected_class_cnt = detected_class_cnt + 1
                        # if self._save_img:
                        #     if not os.path.exists(IMGPATH+"/Segmented_img/%d" %self.i):
                        #         os.makedirs(IMGPATH+"/Segmented_img/%d" %self.i)
                        #     filename = IMGPATH+"Segmented_img/%d/%d_%d.png" %(self.i, self.i, j)       
                        #     cv2.imwrite(os.path.join("", filename), mask_data["segmentation"] * 255)
                    else:
                        if self.debug_mode:
                            print(f"(out of range)mask_data['area'] is {mask_data['area']:.3f}")
                            print("Out of range! - mask min, max area ")
                else:
                    if self.debug_mode:
                        print(f"(No size filtering) mask_data['area'] is {mask_data['area']:.3f}")
                    masks.append(mask_data['segmentation'])
                    masks_area.append(mask_data['area'])
                    boxes.append(box)
                    detected_class_cnt = detected_class_cnt + 1
                    # if self._save_img:
                    #     if not os.path.exists(IMGPATH+"/Segmented_img/%d" %self.i):
                    #         os.makedirs(IMGPATH+"/Segmented_img/%d" %self.i)
                    #     filename = IMGPATH+"Segmented_img/%d/%d_%d.png" %(self.i, self.i, j)        
                    #     cv2.imwrite(os.path.join("", filename), mask_data["segmentation"] * 255)

            else: 
                print(f"#{j:d} Box size out of range!")
                
            # print("object index #", i+1, class_name)
        # print("detected_class_cnt: ", detected_class_cnt)

        if detected_class_cnt > 0:
            start_filter = time.time()
            result_masks, result_masks_area, result_boxes, result_msg.class_cnt = self.filtering(self.i, masks, masks_area, boxes, detected_class_cnt)  
            result_msg.boxes = result_boxes
            print("Object name: ", self._class_names[1] )
            print("Detected_class_cnt: ", result_msg.class_cnt)

            if result_msg.class_cnt != 0:
                mask = Image()
                for result_mask in result_masks:
                    result_data = result_mask.astype(np.uint8) * 255
                    mask = self._cv_bridge.cv2_to_imgmsg(result_data, 'mono8')
                    result_msg.masks.append(mask)
                
                ratio_sorted = sorted(zip(self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_masks), key=lambda x: x[0])
                # _, result_msg.boxes, result_msg.masks = zip(*ratio_sorted)
                # print(self.ratio_set)
                self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_masks = zip(*ratio_sorted)
                end_filter = time.time()
                print(f"filtering runtime: {end_filter - start_filter:.5f} sec")
                idx_tmp = 0

                self.is_multiple_masks = []
                for result_mask in result_msg.masks:
                    if self.debug_mode:
                        print("Selected object index #" , idx_tmp + 1, f"(ratio_set: {self.ratio_set[idx_tmp]:.3f})", f"(area: {result_masks_area[idx_tmp]:d})")

                    ############################
                    #### is_multiple_masks
                    ############################
                    area_tmp = result_masks_area[idx_tmp]
                    # if area_tmp > self._min_area and area_tmp < 1.5*self._mean_area:
                    if area_tmp > self._min_area and area_tmp < self._max_area:
                        self.is_multiple_masks.append(False)
                        if self.debug_mode:
                            print(f"(Single Mask) the area is {area_tmp:.3f}")
                    # elif area_tmp >= 1.5*self._mean_area and area_tmp < 8.0*self._mean_area:
                    # elif area_tmp >= 1.5*self._mean_area and area_tmp < 8.0*self._max_area:
                    elif area_tmp >= 1.5*self._max_area and area_tmp < 8.0*self._max_area:
                        self.is_multiple_masks.append(True)
                        if self.debug_mode:
                            print(f"(Multiple Masks)the area is {area_tmp:.3f}")

                    ############################
                    ############################
                    ############################

                    img_mask = self._cv_bridge.imgmsg_to_cv2(result_mask, 'bgr8')

                    idx_tmp = idx_tmp + 1
                    if self._save_img:
                        if not os.path.exists(IMGPATH+"/Segmented_img/%d" %self.i):
                            os.makedirs(IMGPATH+"/Segmented_img/%d" %self.i)
                        filename = IMGPATH+"Segmented_img/%d/%d_%d.png" %(self.i, self.i, idx_tmp)  
                        cv2.imwrite(os.path.join("", filename), img_mask)

                    if self._do_classification:
                        
                        seg_mask = cv2.resize(img_mask, (512,512))
                        mask_tensor = self.transform(seg_mask).to("cuda:0")
                        if mask_tensor.dim() == 3:
                            mask_tensor = mask_tensor[0]
                        combined_tensor = torch.cat((img_tensor, mask_tensor.unsqueeze(0)), dim=0).unsqueeze(0).to("cuda:0")
                        outputs = self.classifier(combined_tensor)
                        pred_label = torch.argmax(outputs, dim=1).item()
                        result_msg.class_ids.append(pred_label)

                    else:
                        class_id = 1 # SAM은 따로 id가 없음
                        result_msg.class_ids.append(class_id)

                # 근접도 최소값: 1.0
                result_msg.nearby_ratio_set = self.ratio_set   
                # 중복 마스크 여부
                # result_msg.is_multiple_masks_set = self.is_multiple_masks

            else:
                idx_tmp = 0
                result_masks = []
                for i, mask_data in enumerate(result):
                    mask_tmp = mask_data["segmentation"].astype(np.uint8)
                    mask_tmp_sum = mask_tmp.sum()
                    print(f"(No detected mask) #{idx_tmp:d} mask_data['area'] is {mask_data['area']:d}, mask_tmp_sum: {mask_tmp_sum:d}")
                    idx_tmp = idx_tmp + 1 
        else:
            result_msg.class_cnt = 0
            result_masks = []        

        return result_msg, result_masks


    def _build_no_detection_mode_result_msg(self, msg, img_sam, result):
        result_msg = Resultsam()
        result_msg.header = msg.header
        detected_class_cnt = 0
        print('_build_no_detection_mode_result_msg!!!')
        masks = []
        masks_area = []
        boxes = []

        if self.debug_mode:
            for i, mask_data in enumerate(result):
                print(f"(Raw data) mask_data[{i:d}]['area'] is {mask_data['area']:.3f}")

        for j, mask_data in enumerate(result):
            # mask_data["bbox"]: the boundary box detection in xywh format
            box = RegionOfInterest()
            box.x_offset = int(mask_data["bbox"][0]) # bbox_x0, 픽셀 좌표계 기준 min_x value
            box.y_offset = int(mask_data["bbox"][1]) # bbox_y0, 픽셀 좌표계 기준 min_y value
            box.width = int(mask_data["bbox"][2]) # bbox_w
            box.height = int(mask_data["bbox"][3]) # bbox_h

            # class_id = masks['class_ids'][i]        

            score = mask_data["predicted_iou"] # the model's own prediction for the quality of the mask
            # score = mask_data["stability_score"] # an additional measure of mask quality
            result_msg.scores.append(score)

            # No detetion - Only One Mask
            masks.append(mask_data['segmentation'])
            masks_area.append(mask_data['area'])
            boxes.append(box)
            detected_class_cnt = detected_class_cnt + 1

            if self._save_img:
                if not os.path.exists(IMGPATH+"/raw_img/%d" %self.i):
                    os.makedirs(IMGPATH+"/raw_img/%d" %self.i)
                print(f"raw_img #{j:d}, raw_mask_sum is {mask_data['area']:.3f}")
                filename = IMGPATH+"/raw_img/%d/%d_%d.png" %(self.i, self.i, j)
                cv2.imwrite(os.path.join("", filename), mask_data["segmentation"] * 255)

            # print("object index #", i+1, class_name)
        # print("detected_class_cnt: ", detected_class_cnt)

        if detected_class_cnt > 0:
            start_filter = time.time()
            result_masks, result_masks_area, result_boxes, result_msg.class_cnt = self.filtering(self.i, masks, masks_area, boxes, detected_class_cnt)  
            result_msg.boxes = result_boxes
            print("Object name: ", self._class_names[1] )
            print("Detected_class_cnt: ", result_msg.class_cnt)

            if result_msg.class_cnt != 0:
                mask = Image()
                for result_mask in result_masks:
                    result_data = result_mask.astype(np.uint8) * 255
                    mask = self._cv_bridge.cv2_to_imgmsg(result_data, 'mono8')
                    result_msg.masks.append(mask)
                
                ratio_sorted = sorted(zip(self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_masks), key=lambda x: x[0])
                # _, result_msg.boxes, result_msg.masks = zip(*ratio_sorted)
                # print(self.ratio_set)
                self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_masks = zip(*ratio_sorted)
                end_filter = time.time()
                print(f"filtering runtime: {end_filter - start_filter:.5f} sec")
                idx_tmp = 0

                self.is_multiple_masks = []
                for result_mask in result_msg.masks:
                    if self.debug_mode:
                        print("Selected object index #" , idx_tmp + 1, f"(ratio_set: {self.ratio_set[idx_tmp]:.3f})", f"(area: {result_masks_area[idx_tmp]:d})")

                    ############################
                    #### is_multiple_masks
                    ############################
                    area_tmp = result_masks_area[idx_tmp]
                    # if area_tmp > self._min_area and area_tmp < 1.5*self._mean_area:
                    if area_tmp > self._min_area and area_tmp < self._max_area:
                        self.is_multiple_masks.append(False)
                        if self.debug_mode:
                            print(f"(Single Mask) the area is {area_tmp:.3f}")
                    # elif area_tmp >= 1.5*self._mean_area and area_tmp < 8.0*self._mean_area:
                    # elif area_tmp >= 1.5*self._mean_area and area_tmp < 8.0*self._max_area:
                    elif area_tmp >= 1.5*self._max_area and area_tmp < 8.0*self._max_area:
                        self.is_multiple_masks.append(True)
                        if self.debug_mode:
                            print(f"(Multiple Masks)the area is {area_tmp:.3f}")

                    ############################
                    ############################
                    ############################

                    img_mask = self._cv_bridge.imgmsg_to_cv2(result_mask, 'bgr8')

                    idx_tmp = idx_tmp + 1
                    if self._save_img:
                        if not os.path.exists(IMGPATH+"/Segmented_img/%d" %self.i):
                            os.makedirs(IMGPATH+"/Segmented_img/%d" %self.i)
                        filename = IMGPATH+"Segmented_img/%d/%d_%d.png" %(self.i, self.i, idx_tmp)  
                        cv2.imwrite(os.path.join("", filename), img_mask)


                    class_id = 1 # SAM은 따로 id가 없음
                    result_msg.class_ids.append(class_id)

                        

                # 근접도 최소값: 1.0
                result_msg.nearby_ratio_set = self.ratio_set   
                # 중복 마스크 여부
                # result_msg.is_multiple_masks_set = self.is_multiple_masks

            else:
                idx_tmp = 0
                result_masks = []
                for i, mask_data in enumerate(result):
                    mask_tmp = mask_data["segmentation"].astype(np.uint8)
                    mask_tmp_sum = mask_tmp.sum()
                    print(f"(No detected mask) #{idx_tmp:d} mask_data['area'] is {mask_data['area']:d}, mask_tmp_sum: {mask_tmp_sum:d}")
                    idx_tmp = idx_tmp + 1 
        else:
            result_msg.class_cnt = 0
            result_masks = []        

        return result_msg, result_masks


    def size_filter(self, masks, class_cnt):           
        del_idx = []
        if self._is_mean_area_assigned:
            mean_size = self._mean_area
            print(f"Assigned mean size is {mean_size:d}")
        else:
            mean_size = (self.whole_size / class_cnt) # json에 저장되어야 하는 평균 크기 입니다. (민우형에게)
            print(f"Current mean size is {mean_size:.3f}")

        
        for i in range(class_cnt):
            if (self.size_set[i] < (mean_size * min_size_thre) or self.size_set[i] > (mean_size * max_size_thre)):
                del_idx.append(i)

        for i in del_idx:
            if self._save_img:
                filename = "/home/" + username_ + "/[zivid_scan_data]/Deleted_img/size-%d.png" % i
                cv2.imwrite(os.path.join("", filename), masks[i]*255)  
            
        return del_idx        


    def filtering(self, seq, masks, masks_area, boxes, class_cnt):      
        del_idx = []
        ratio_set = []
        bb_margin = 40 # zivid는 1pixel에 대략 0.37mm, 40 pixel = 14.8 mm  --> bounding box 의 경계에서 약 15mm만큼 padding

        #### a) Overlap을 이용하여 SAM에서 찾지 못한 단일 마스크를 추가
        if self._do_additional_processing:
            print('Additional processing!')
            class_cnt_tmp = class_cnt
            for i in range(class_cnt_tmp):
                mask1 = masks[i].astype(np.uint8)
                mask1_sum = masks_area[i]

                # print(masks[i])

                # Margin
                bbx1 = boxes[i].x_offset - bb_margin
                bby1 = boxes[i].y_offset - bb_margin
                bbw1 = boxes[i].width + 2*bb_margin
                bbh1 = boxes[i].height + 2*bb_margin

                for j in range(i+1, class_cnt_tmp):   
                    # Margin
                    bbx2 = boxes[j].x_offset - bb_margin
                    bby2 = boxes[j].y_offset - bb_margin
                    bbw2 = boxes[j].width + 2*bb_margin
                    bbh2 = boxes[j].height + 2*bb_margin

                    if (bbx1<=bbx2+bbw2 and bby1<=bby2+bbh2 and bbx1+bbw1>=bbx2 and bby1+bbh1>=bby2):
                        mask2 = masks[j].astype(np.uint8)
                        mask2_sum = masks_area[j]

                        overlap = self.overlap_ratio(mask1, mask2, mask1_sum, mask2_sum)              
                        if (overlap > self._overlap_threshold):
                            if (mask1_sum) > (mask2_sum):
                                # absdiff 함수를 사용하여 이미지 간의 차이 계산
                                diff = cv2.absdiff(mask1*255, mask2*255)
                                mask_diff = diff.astype(np.bool_)
                                mask_diff_sum = mask_diff.astype(np.uint8).sum()
                                # update
                                masks.append(mask_diff)
                                masks_area.append(mask_diff_sum)
                                boxes.append(boxes[i])
                                class_cnt = class_cnt +1

                                # # 결과 이미지 출력
                                # cv2.imshow('mask1', mask1*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()
                                # cv2.imshow('mask2', mask2*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()
                                # mask_diff_tmp = mask_diff.astype(np.uint8)
                                # cv2.imshow('Difference 1', mask_diff_tmp*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()
                            else:
                                # absdiff 함수를 사용하여 이미지 간의 차이 계산
                                diff = cv2.absdiff(mask2*255, mask1*255)
                                mask_diff = diff.astype(np.bool_)
                                mask_diff_sum = mask_diff.astype(np.uint8).sum()
                                # update
                                masks.append(mask_diff)
                                masks_area.append(mask_diff_sum)
                                boxes.append(boxes[j])
                                class_cnt = class_cnt +1
                                # # 결과 이미지 출력
                                # cv2.imshow('mask1', mask1*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()
                                # cv2.imshow('mask2', mask2*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()
                                # mask_diff_tmp = mask_diff.astype(np.uint8)
                                # cv2.imshow('Difference 2', mask_diff_tmp*255)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()          

        ################################################
        #### b) 윤택's 방법, overlap & size filter 적용하여 우선순위 설정 및 주변에 물체로 가려진 것은 인덱스에서 제외
        if self._do_overlap_filtering:
            
            bb_margin = 0

            print(f"(Process A) class_cnt: {class_cnt:d}")
            for i in range(class_cnt):
                mask1 = masks[i].astype(np.uint8)
                # mask1_sum = mask1.sum()
                mask1_sum = masks_area[i]
                
                # print(f"mask1_sum is {mask1_sum:.3f}")
                # print(f"masks_area[i] is {masks_area[i]:.3f}")

                # Margin
                bbx1 = boxes[i].x_offset - bb_margin
                bby1 = boxes[i].y_offset - bb_margin
                bbw1 = boxes[i].width + 2*bb_margin
                bbh1 = boxes[i].height + 2*bb_margin

                self.whole_size += mask1_sum
                self.size_set.append(mask1_sum)
                kernel = np.ones((3,3),np.uint8)           
                dilated_mask1 = cv2.dilate(mask1, kernel, iterations=nearby_radius)           
                ratio_set.append([])

                for j in range(i+1):     

                    # Margin
                    bbx2 = boxes[j].x_offset - bb_margin
                    bby2 = boxes[j].y_offset - bb_margin
                    bbw2 = boxes[j].width + 2*bb_margin
                    bbh2 = boxes[j].height + 2*bb_margin

                    ratio = 0
                    if (bbx1<=bbx2+bbw2 and bby1<=bby2+bbh2 and bbx1+bbw1>=bbx2 and bby1+bbh1>=bby2):
                        mask2 = masks[j].astype(np.uint8) 
                        dilated_mask2 = cv2.dilate(mask2, kernel, iterations=nearby_radius)             
                        ratio = self.nearby_ratio(dilated_mask1, dilated_mask2)                
                        if ((j in self.overlap_pair and i == del_idx[self.overlap_pair.index(j)])
                            or (i in self.overlap_pair and j == del_idx[self.overlap_pair.index(i)])):
                            ratio = 0
                    ratio_set[i].append(ratio)

                    # print(f"ratio A [{i}/{j}] : {ratio}") 

                for j in range(i+1, class_cnt):   
                    # Margin
                    bbx2 = boxes[j].x_offset - bb_margin
                    bby2 = boxes[j].y_offset - bb_margin
                    bbw2 = boxes[j].width + 2*bb_margin
                    bbh2 = boxes[j].height + 2*bb_margin

                    ratio = 0
                    if (bbx1<=bbx2+bbw2 and bby1<=bby2+bbh2 and bbx1+bbw1>=bbx2 and bby1+bbh1>=bby2):
                        mask2 = masks[j].astype(np.uint8) 
                        # mask2_sum = mask2.sum()
                        mask2_sum = masks_area[j]
                        # print(f"mask2_sum is {mask2_sum:.3f}")
                        # print(f"masks_area[j] is {masks_area[j]:.3f}")

                        dilated_mask2 = cv2.dilate(mask2, kernel, iterations=nearby_radius)
                        ratio = self.nearby_ratio(dilated_mask1, dilated_mask2)

                        #### overlap = intersection/smaller area
                        overlap = self.overlap_ratio(mask1, mask2, mask1_sum, mask2_sum)

                        if (overlap > self._overlap_threshold):
                            if (mask1_sum) < (mask2_sum):
                                print(f"mask2(area: {mask2_sum}) is overlap mask of mask1(area: {mask1_sum})")
                                del_idx.append(j)     
                                self.overlap_pair.append(i)     
                            else:
                                print(f"mask1(area: {mask1_sum}) is overlap mask of mask2(area: {mask2_sum})")
                                del_idx.append(i)     
                                self.overlap_pair.append(j)      
                            ratio = 0



                    ratio_set[i].append(ratio)
                    
                    # print(f"ratio B [{i}/{j}] : {ratio}")            

            ############################# double overlap filter ########################
            # l = []
            # overlap_del_idx = []
            # for i in del_idx:
            #     if i not in l:
            #         l.append(i)
            #     else:
            #         if i not in overlap_del_idx:
            #             overlap_del_idx.append(i)

            # result_del_idx = overlap_del_idx

            ############################# mean size filter #############################
            # for i in del_idx:        
            #     if self._save_img:    
            #         filename = "/home/" + username_ + "/[zivid_scan_data]/Deleted_img/overlap-%d.png" % i
            #         cv2.imwrite(os.path.join("", filename), masks[i]*255)  
            #     self.whole_size -= masks[i].sum()

            # size_del_idx = self.size_filter(masks, class_cnt)
            # del_idx_search = copy.deepcopy(del_idx)        

            # for i in size_del_idx:                    
            #     if i in self.overlap_pair:
            #         del_idx.remove(del_idx_search[self.overlap_pair.index(i)])
            # overlap_del_idx = list(set(del_idx))

            # result_del_idx = size_del_idx + overlap_del_idx
            
            result_del_idx = del_idx
            result_del_idx = list(set(result_del_idx))
            result_del_idx = sorted(result_del_idx, reverse=True)
            class_cnt = class_cnt - len(result_del_idx)

            for i in result_del_idx:
                if self._save_img:    
                    if not os.path.exists(IMGPATH+"/Deleted_img/%d" %seq):
                        os.makedirs(IMGPATH+"/Deleted_img/%d" %seq)
                    filename = IMGPATH+"Deleted_img/%d/%d.png" %(seq, i) 
                    cv2.imwrite(os.path.join("", filename), masks[i]*255)   
                del masks[i]
                del masks_area[i]
                del boxes[i]
                del ratio_set[i]

            #### size 지정
            if self._do_additional_processing:
                mask_cnt = 0
                print("*********************************************************")
                # print(len(masks_area))
                # print(class_cnt)
                mask_area_del_idx = []
                for i in range(class_cnt):
                    if masks_area[i] > self._min_area and masks_area[i] < self._max_area:
                        # print(f"(in range)mask_data['area'] is {masks_area[i]:d}")
                        mask_cnt = mask_cnt + 1
                    else:
                        mask_area_del_idx.append(i)


                mask_area_del_idx = list(set(mask_area_del_idx))
                mask_area_del_idx = sorted(mask_area_del_idx, reverse=True)
                class_cnt = class_cnt - len(mask_area_del_idx)

                for i in mask_area_del_idx:
                    print(f"(out of range)mask_data['area'] is {masks_area[i]:d}")
                    print("Out of range! - mask min, max area ")
                    del masks[i]
                    del masks_area[i]
                    del boxes[i]
                    del ratio_set[i]

                
                print("*********************************************************\n")
            else:
                mask_cnt = class_cnt

            for i in range(mask_cnt):
                # for j in size_del_idx:               
                #     ratio_set[i][j] = 0
                self.ratio_set.append(sum(ratio_set[i][:]))


        else:
            print('No overlap Filtering!!!')
            mask_cnt = class_cnt
            for i in range(mask_cnt):
                self.ratio_set.append(1.0)


        return masks, masks_area, boxes, class_cnt

    def nearby_ratio(self, mask1, mask2):
        intersection = (mask1 * mask2).sum()
        if intersection == 0:
            return 0.0
        return intersection / mask1.sum()

    def overlap_ratio(self, mask1, mask2, mask1_sum, mask2_sum):
        intersection = (mask1 * mask2).sum()
        if intersection == 0:
            return 0.0
        small_mask = min (mask1_sum, mask2_sum)
        return intersection / small_mask

    def _class_filter(self, result_msg):
        
        print("boxes_len:", len(result_msg.boxes))
        print("class_ids_len:", len(result_msg.class_ids))
        print("class_names_len:", len(result_msg.class_names))
        print("scores_len:", len(result_msg.scores))
        print("masks_len:", len(result_msg.masks))
        print("nearby_ratio_set_len:", len(result_msg.nearby_ratio_set))
        class_id = 4 #else
        filtered_boxes = []
        filtered_class_ids = []
        filtered_class_names = []
        filtered_masks = []
        filtered_nearby_ratio_set = []

        for i in range(len(result_msg.masks)):
            if self._class_names[1] == 'bolt_bush':
                class_id = 0
            elif self._class_names[1] == 'peg':
                class_id = 1
            elif self._class_names[1] == 'square_peg':
                class_id = 2
            elif self._class_names[1] == 't_joint':
                class_id = 3

            if result_msg.class_ids[i] == class_id:
                filtered_boxes.append(result_msg.boxes[i])
                filtered_class_ids.append(result_msg.class_ids[i])
                filtered_class_names.append(self._class_names[1])
                filtered_masks.append(result_msg.masks[i])
                filtered_nearby_ratio_set.append(result_msg.nearby_ratio_set[i])
            
        filtered_msg = Resultsam()
        filtered_msg.header = result_msg.header
        filtered_msg.boxes = filtered_boxes
        filtered_msg.class_ids = filtered_class_ids
        filtered_msg.class_names = filtered_class_names
        filtered_msg.masks = filtered_masks
        filtered_msg.nearby_ratio_set = filtered_nearby_ratio_set
        filtered_msg.class_cnt = len(filtered_masks)

        return filtered_msg

    def _visualize(self, masks, image, result_msg, result_masks):

        mask_img = self.show_anns(masks)
        mask_img = cv2.normalize(mask_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        mask_img = np.invert(mask_img)

        background_img = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)   

        overlay_image = cv2.addWeighted(background_img,1.0,mask_img,0.5,0, dtype = cv2.CV_8U)

        target_num = 0
        font = cv2.FONT_HERSHEY_SIMPLEX

        for i in range(len(result_masks)):            
            class_id = result_msg.class_ids[i]
            if class_id == 0:
                class_ = "bolt_bush"
                color_ = (255, 0, 0)
            elif class_id == 1:
                class_ = "peg"
                color_ = (0, 255, 0)
            elif class_id == 2:
                class_ = "square_peg"
                color_ = (0, 0, 255)
            elif class_id == 3:
                class_ = "t_joint"
                color_ = (255, 255, 0)
            elif class_id == 4:
                class_ = "else"
                color_ = (0, 0, 0)
            
            mask = result_masks[i].astype(np.uint8)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay_image, contours, -1, color_, 5, cv2.LINE_8, hierarchy, 0)
            
            if self._do_classification:
                if class_ == self._class_names[1]:
                    target_num += 1
                    text_ = f"{target_num}"
                    overlay_image = cv2.putText(overlay_image, text_, (result_msg.boxes[i].x_offset + int(result_msg.boxes[i].width/2), 
                                            result_msg.boxes[i].y_offset + int(result_msg.boxes[i].height/2)), font, 1.2, (0, 0, 0), 3)
                if class_ == "peg":
                    class_ = "cylinder_peg"

                overlay_image = cv2.putText(overlay_image, class_, (result_msg.boxes[i].x_offset, 
                                        result_msg.boxes[i].y_offset + int(result_msg.boxes[i].height/1.5)), font, 1.0, color_, 2)
            else:
                target_num += 1
                text_ = f"{target_num}"
                overlay_image = cv2.putText(overlay_image, text_, (result_msg.boxes[i].x_offset + int(result_msg.boxes[i].width/2), 
                                        result_msg.boxes[i].y_offset + int(result_msg.boxes[i].height/2)), font, 1.2, (0, 0, 0), 3)


        return overlay_image
    
    def _image_callback(self, msg):
        self.node.get_logger().debug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def _load_weight_callback(self, msg):
        self.node.get_logger().debug("Get a command for loading weight")
        if self._msg_lock_2.acquire(False):
            self._last_msg_2 = msg
            self._msg_lock_2.release()

def main():
    rclpy.init(args=sys.argv)

    node = SamNode()
    node.run()

if __name__ == '__main__':
    main()
