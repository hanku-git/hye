#!/usr/bin/env python3

import numpy as np
import copy
import math
import cv2
import sys
import time
import json
import os
import threading
import rclpy

from cv_bridge import CvBridge

from bolt_classifier  import BoltModel
from hinge_classifier import HingeModel
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose2D
from hanyang_matching_msgs.msg import Resultsam
from hanyang_matching_msgs.msg import BpUiCommandLearning


BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))
IMGPATH = BASE_DIR + "/[zivid_scan_data]/"
sam_checkpoint = BASE_DIR + "/[sam_weight]/sam_vit_h_4b8939.pth"
model_type = "vit_h"
device = "cuda"
test = False
# save_img = True
min_size_thre = 0.8 
max_size_thre = 6.0
# overlap_thre = 0.9
nearby_radius = 10

CLASS_NAMES = ['BG','default']

class SamNode(object):
    def __init__(self):
        self.node = rclpy.create_node('sam_zivid')
        self._cv_bridge = CvBridge()
        self.BM = BoltModel()
        self.HM = HingeModel()
        self.init_set()
        self._publish_rate = self.node.declare_parameter('~publish_rate', 100).value

        #### SAM
        print("Loading model...")
        self.sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.sam.to(device=device)
        self._target_name = 'default'
        self.setSamParameters()

        self._visualization = self.node.declare_parameter('~visualization', True)
        print("Loading finished!")

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
        with open (BASE_DIR + "/sam_ws/src/sam_ros/config/sam_config.json", "r") as f:
            self.sam_config = json.load(f)
            print("-------------------\n")
            print("--- sam_config ----\n")
            print("Target object: ", self._target_name)
            print("-------------------\n",  f"(points_per_side: {self.sam_config[self._target_name]['points_per_side']:d})\n", 
                                            f"(points_per_batch: {self.sam_config[self._target_name]['points_per_batch']:d})\n",
                                            f"(pred_iou_thresh: {self.sam_config[self._target_name]['pred_iou_thresh']:.2f})\n",
                                            f"(stability_score_thresh: {self.sam_config[self._target_name]['stability_score_thresh']:.2f})\n",
                                            f"(min_mask_region_area: {self.sam_config[self._target_name]['min_mask_region_area']:d})\n",
                                            f"\n(min_area: {self.sam_config[self._target_name]['min_area']:d})\n",
                                            f"\n(min_area: {self.sam_config[self._target_name]['min_area']:d})\n",
                                            f"(max_area: {self.sam_config[self._target_name]['max_area']:d})\n",
                                            f"(mean_area: {self.sam_config[self._target_name]['mean_area']:d})\n")

            print("-------------------\n")
        output_mode = "binary_mask" # "coco_rle", "binary_mask", "uncompressed_rle"
        self.mask_generator = SamAutomaticMaskGenerator(
            model=self.sam,
            output_mode=output_mode,

            points_per_side         = self.sam_config[self._target_name]['points_per_side'],
            points_per_batch        = self.sam_config[self._target_name]['points_per_batch'],
            pred_iou_thresh         = self.sam_config[self._target_name]['pred_iou_thresh'],
            stability_score_thresh  = self.sam_config[self._target_name]['stability_score_thresh'],
            stability_score_offset  = self.sam_config[self._target_name]['stability_score_offset'],

            box_nms_thresh=0.7,
            crop_n_layers=0,
            crop_nms_thresh=0.7,
            crop_overlap_ratio=1500 / 1500,
            crop_n_points_downscale_factor=1,
            # min_mask_region_area=6000,  # Requires open-cv to run post-processing
            min_mask_region_area = self.sam_config[self._target_name]['min_mask_region_area'],  # Requires open-cv to run post-processing
        )
        self._min_area = self.sam_config[self._target_name]['min_area']
        self._max_area = self.sam_config[self._target_name]['max_area']
        self._mean_area = self.sam_config[self._target_name]['mean_area']
        self._is_mean_area_assigned = self.sam_config[self._target_name]['is_mean_area_assigned']
        self._do_additional_processing = self.sam_config[self._target_name]['do_additional_processing']
        self._do_classification = self.sam_config[self._target_name]['do_classification']

        self._overlap_threshold = self.sam_config[self._target_name]['overlap_threshold']
        self.border_threshold = self.sam_config[self._target_name]['border_threshold']
        self._save_img = self.sam_config[self._target_name]['save_img']

        self._do_min_max_size_filtering = self.sam_config[self._target_name]['do_min_max_size_filtering']
        self._do_overlap_filtering = self.sam_config[self._target_name]['do_overlap_filtering']
        
        self.bin_x_min = self.sam_config[self._target_name]['bin_area'][0]
        self.bin_x_max = self.sam_config[self._target_name]['bin_area'][1]
        self.bin_y_min = self.sam_config[self._target_name]['bin_area'][2]
        self.bin_y_max = self.sam_config[self._target_name]['bin_area'][3]
        self.debug_mode = self.sam_config[self._target_name]['debug_mode']

        if self.debug_mode:
            print("debug_mode!!!")

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
                print('[sam_node_hyundai] image input!')
                start = time.time()
                np_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.init_set()
                # Run detection
                start_sam = time.time()
                h_, w_ = np_image.shape[0:2]
                print ("image shape: ", h_,", ",w_)
                self.img_sam = np_image[self.bin_y_min:self.bin_y_max, self.bin_x_min:self.bin_x_max]
                margin = [[self.bin_y_min, h_-self.bin_y_max],[self.bin_x_min, w_-self.bin_x_max]]
                print("margin: ", margin)
                if self._save_img:                    
                    filename = IMGPATH + "%d.png" % self.i
                    cv2.imwrite(os.path.join("", filename), self.img_sam)

                masks = self.mask_generator.generate(self.img_sam)
                end_sam = time.time()      
                print(f"sam detection runtime: {end_sam - start_sam:.5f} sec")    

                result_msg, vis_masks, vis_boxes, vis_poses = self._build_result_msg(msg, masks, margin)

                self._result_pub.publish(result_msg)                

                if result_msg.class_cnt == 0 :
                    print('No mask detected!')
                else :
                    print('Detection!')
                    print(f"Dectected Mask #: {len(result_msg.masks)}")
                    print(f"published!! mask number: {len(result_msg.masks)}")                                 

                # Visualize results
                if self._visualization:
                    vis_image = self._visualize(masks, self.img_sam, vis_masks, vis_boxes, vis_poses)
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
                self._target_name = msg.target_name
                self.setSamParameters()
                print('Parameter initialize! (object: ' + msg.target_name + ')')
                if self._is_mean_area_assigned:
                    print(f"Assigned mean size is {self._mean_area:d}")
                else:
                    print("Mean size should be assigned!")

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

    def _build_result_msg(self, msg, result, margin):
        result_msg = Resultsam()
        result_msg.header = msg.header
        detected_class_cnt = 0
        print('_build_result_msg!!!')
        masks = []
        masks_area = []
        boxes = []
        vis_poses = []
        
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
                
            print(f"#{j:d} x:{box.x_offset} y: {box.y_offset}")
            if (box.width <= (self.bin_x_max - self.bin_x_min)* self.border_threshold and
                box.height <= (self.bin_y_max - self.bin_y_min)* self.border_threshold):

                print(f"#{j:d} Box size in range!")

                if self._save_img:
                    if not os.path.exists(IMGPATH+"/raw_img/%d" %self.i):
                        os.makedirs(IMGPATH+"/raw_img/%d" %self.i)
                    print(f"raw_img #{j:d}, raw_mask_sum is {mask_data['area']:.3f}")
                    filename = IMGPATH+"/raw_img/%d/%d_%d.png" %(self.i, self.i, j)
                    cv2.imwrite(os.path.join("", filename), mask_data["segmentation"] * 255)

                if self._do_min_max_size_filtering:
                    if mask_data['area'] > self._min_area and mask_data['area'] < self._max_area:
                        if self.debug_mode:
                            print(f"(in range)mask_data['area'] is {mask_data['area']:.3f}")
                        masks.append(mask_data['segmentation'])
                        masks_area.append(mask_data['area'])
                        boxes.append(box)
                        detected_class_cnt = detected_class_cnt + 1
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

            else: 
                print(f"#{j:d} Box size out of range!")

        if detected_class_cnt > 0:
            start_filter = time.time()

            result_masks, result_masks_area, result_boxes, result_class_cnt = self.filtering(self.i, masks, masks_area, boxes, detected_class_cnt)  
            if self._target_name == "bolt":
                result_masks, result_masks_area, result_boxes, result_msg.picking_poses, vis_poses, result_class_cnt = self.bolt_filter(result_masks, result_masks_area, result_boxes, margin)
            elif self._target_name == "hinge":
                result_masks, result_masks_area, result_boxes, result_msg.picking_poses, vis_poses, result_class_cnt = self.hinge_filter(result_masks, result_masks_area, result_boxes, margin)

            result_msg.class_cnt = result_class_cnt
            print("Object name: ", self._class_names[1] )
            print("Detected_class_cnt: ", result_msg.class_cnt)
            vis_masks = result_masks.copy()
            vis_boxes = copy.deepcopy(result_boxes)

            if result_msg.class_cnt != 0:
                mask = Image()
                for i in range(result_msg.class_cnt):
                    result_masks[i] = np.pad(result_masks[i], margin, 'constant')
                    result_data = result_masks[i].astype(np.uint8) * 255
                    mask = self._cv_bridge.cv2_to_imgmsg(result_data, 'mono8')
                    result_boxes[i].x_offset = result_boxes[i].x_offset + margin[1][0]
                    result_boxes[i].y_offset = result_boxes[i].y_offset + margin[0][0]
                    result_msg.boxes.append(result_boxes[i])
                    result_msg.masks.append(mask)
                # bolt의 경우 ratio_set = centroid distance_set
                if self._target_name == "bolt" or self._target_name == "hinge":
                    ratio_sorted = sorted(zip(self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_msg.picking_poses, vis_poses, vis_masks, vis_boxes), key=lambda x: x[0])
                    self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, result_msg.picking_poses, vis_poses, vis_masks, vis_boxes = zip(*ratio_sorted)
                else:
                    ratio_sorted = sorted(zip(self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, vis_masks, vis_boxes), key=lambda x: x[0])
                    self.ratio_set, result_msg.boxes, result_msg.masks, result_masks_area, vis_masks, vis_boxes = zip(*ratio_sorted)
                end_filter = time.time()
                print(f"filtering runtime: {end_filter - start_filter:.5f} sec")
                idx_tmp = 0

                self.is_multiple_masks = []
                for result_mask in result_msg.masks:
                    if self.debug_mode:
                        # print("Selected object index #" , idx_tmp + 1, f"\n(area: {result_masks_area[idx_tmp]:d})", f"(angle: {result_msg.picking_poses[idx_tmp].theta:f})")
                        print("Selected object index #" , idx_tmp + 1, f"\n(area: {result_masks_area[idx_tmp]:d})")
                    img_mask = self._cv_bridge.imgmsg_to_cv2(result_mask, 'mono8')

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
                vis_masks = []
                vis_boxes = []
                result_msg.class_ids.append(1)
                for i, mask_data in enumerate(result):
                    mask_tmp = mask_data["segmentation"].astype(np.uint8)
                    mask_tmp_sum = mask_tmp.sum()
                    print(f"(No detected mask) #{idx_tmp:d} mask_data['area'] is {mask_data['area']:d}, mask_tmp_sum: {mask_tmp_sum:d}")
                    idx_tmp = idx_tmp + 1 
        else:
            result_msg.class_cnt = 0 
            result_msg.class_ids.append(1)
            vis_masks = []
            vis_boxes = []    

        return result_msg, vis_masks, vis_boxes, vis_poses

    def filtering(self, seq, masks, masks_area, boxes, class_cnt):      
        del_idx = []
        ratio_set = []
        bb_margin = 40 # zivid는 1pixel에 대략 0.37mm, 40 pixel = 14.8 mm  --> bounding box 의 경계에서 약 15mm만큼 padding

        if self._do_overlap_filtering:
            
            bb_margin = 0

            print(f"(Process A) class_cnt: {class_cnt:d}")
            for i in range(class_cnt):
                mask1 = masks[i].astype(np.uint8)
                mask1_sum = masks_area[i]

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
                        mask2_sum = masks_area[j]

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

            ########################## double overlap filter ########################
            # l = []
            # overlap_del_idx = []
            # for i in del_idx:
            #     if i not in l:
            #         l.append(i)
            #     else:
            #         if i not in overlap_del_idx:
            #             overlap_del_idx.append(i)

            # result_del_idx = overlap_del_idx
            
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

    def bolt_filter(self, masks, masks_area, boxes, margin):  
        bm_ = self.BM       
        result_poses = []
        vis_poses = []
        result_masks = []
        result_boxes = []
        result_masks_area =[]
        self.ratio_set = [] # bolt의 경우 centroid distance
        h_, w_ = self.img_sam.shape[0:2]
        rgb_resized_region = bm_.extract_rgb_masks(self.img_sam, masks)
        index_ = bm_.get_index(rgb_resized_region)
        indices_of_zeros = [idx for idx, value in enumerate(index_) if value == 0] # 0이 존재하는 인덱스들
        for i in indices_of_zeros:
            pose = Pose2D()
            vis_pose = Pose2D()
            mask = masks[i]
            bx = boxes[i].x_offset
            by = boxes[i].y_offset
            bw = boxes[i].width
            bh = boxes[i].height            
            masked_region = mask[by:by+bh, bx:bx+bw]
            major_axis, minor_axis = bm_.find_major_minor_axes(masked_region)

            # Segment the bolt into head and tail
            head_points, tail_points = bm_.segment_bolt_head_tail(masked_region, minor_axis)
            head_centroid = bm_.find_centroid(head_points)
            tail_centroid = bm_.find_centroid(tail_points)
            # Plot the head and tail parts along with major and minor axes
            # If you want to get only picking point, comment out this line

            angle_degrees = bm_.calculate_angle_from_origin_to_vector(head_centroid, tail_centroid)

            point_5_8 = bm_.find_point_on_segment(head_centroid, tail_centroid, 5/8)
            # point_5_8_global = (point_5_8[0] + bx, point_5_8[1] + by)
            # picking_point = [point_5_8_global[0] + margin[0][0], point_5_8_global[1] + margin[1][0]]              
            vis_pose.x = point_5_8[0] + bx
            vis_pose.y = point_5_8[1] + by
            vis_pose.theta = angle_degrees
            distance = math.sqrt((w_/2 - vis_pose.x) ** 2 + (h_/2 - vis_pose.y) ** 2)
            pose.x = point_5_8[0] + bx + margin[1][0]
            pose.y = point_5_8[1] + by + margin[0][0]
            pose.theta = angle_degrees
            vis_poses.append(vis_pose)
            self.ratio_set.append(distance)
            result_poses.append(pose)
            result_masks.append(mask)
            result_boxes.append(boxes[i])
            result_masks_area.append(masks_area[i])
        return result_masks, result_masks_area, result_boxes, result_poses, vis_poses, len(indices_of_zeros)

    def hinge_filter(self, masks, masks_area, boxes, margin):
        hm_ = self.HM      
        is_there_class_0 = False 
        vis_poses = []
        result_poses = []
        result_masks = []
        result_boxes = []
        result_masks_area =[]

        for i, mask in enumerate(masks):
            vis_pose = Pose2D()
            pose = Pose2D()
            class_idx, image = hm_.preprocess_image(self.img_sam, mask)

            if class_idx == 0 and not is_there_class_0:         #class_1: hinge 일부분, class_0: hinge 전체
                is_there_class_0 = True
                result_poses.clear()
                result_masks.clear()
                result_boxes.clear()
                result_masks_area.clear()

            if (is_there_class_0 and class_idx == 0) or (not is_there_class_0 and class_idx == 1):
                angle = hm_.predict_angle(image)
                pose.x = boxes[i].x_offset + boxes[i].width/2 + margin[1][0]
                pose.y = boxes[i].y_offset + boxes[i].height/2 + margin[0][0]
                pose.theta = angle
                vis_pose.x = boxes[i].x_offset + boxes[i].width/2
                vis_pose.y = boxes[i].y_offset + boxes[i].height/2
                vis_pose.theta = angle
                vis_poses.append(vis_pose)
                result_poses.append(pose)
                result_masks.append(mask)
                result_boxes.append(boxes[i])
                result_masks_area.append(masks_area[i])

        return result_masks, result_masks_area, result_boxes, result_poses, vis_poses, len(result_masks)

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

    def _visualize(self, masks, image, vis_masks, vis_boxes, picking_poses):

        mask_img = self.show_anns(masks)
        mask_img = cv2.normalize(mask_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        mask_img = np.invert(mask_img)

        background_img = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)   

        overlay_image = cv2.addWeighted(background_img,1.0,mask_img,0.5,0, dtype = cv2.CV_8U)
        target_num = 0

        font = cv2.FONT_HERSHEY_SIMPLEX
        if self._target_name == "bolt":
            thickness = 2
        else:
            thickness= 5
        for i in range(len(vis_masks)):            
            
            mask = vis_masks[i].astype(np.uint8)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay_image, contours, -1, (255, 0, 0), thickness, cv2.LINE_8, hierarchy, 0)

            if self._target_name == 'bolt' or self._target_name == 'hinge':
                x = int(picking_poses[i].x)
                y = int(picking_poses[i].y)
                angle_rad = math.radians(picking_poses[i].theta)
                end_x = int(x + 25 * math.cos(angle_rad))
                end_y = int(y - 25 * math.sin(angle_rad))
                overlay_image = cv2.arrowedLine(overlay_image, (x, y), (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)
                text_ = f"{i + 1}"
                overlay_image = cv2.putText(overlay_image, text_, (vis_boxes[i].x_offset + int(vis_boxes[i].width/2), 
                                        vis_boxes[i].y_offset + int(vis_boxes[i].height/2)), font, 0.5, (0, 0, 0), 2)

            else:
                target_num += 1
                text_ = f"{target_num}"
                overlay_image = cv2.putText(overlay_image, text_, (vis_boxes[i].x_offset + int(vis_boxes[i].width/2), 
                                        vis_boxes[i].y_offset + int(vis_boxes[i].height/2)), font, 1.0, (0, 0, 0), 3)


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
