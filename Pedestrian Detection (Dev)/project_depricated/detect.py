import time
t = time.time()

import os
import argparse
import cv2
import numpy as np
import onnxruntime as ort
# import picamera2


RHO = 3.5 
THETA = 3*np.pi/180
MIN_VOTES = 15
MIN_LINE_LEN = 15 
MAX_LINE_GAP= 5
MASK_THRESHOLD = 50
MAX_DETECTIONS = 35
CONFIDENCE_THRESHOLD = 0.35
IOU_THRESHOLD = 0.35
FOV = 120

classes_names = [
    'pedestrian',
    'truck',
    'car',
    'cyclist',
    'misc',
    'van',
    'tram',
    'sitting person'
]

def calculate_horizontal_angle(input):
    angle_deg = np.zeros((MAX_DETECTIONS,), dtype=np.float32)
    x_min, _, x_max, _ = np.array(input['boxes']).T
    
    for i in range(input['num_detections'][0]):
        bbox_center_x = (x_min[i] + x_max[i]) / 2
        delta_x = bbox_center_x - 320
        angle_deg[i] = (delta_x / 320) * (FOV / 2)
    return angle_deg


def lane_postprocessing(mask):
    

    def find_edges(img):
        blur = cv2.GaussianBlur(img, (5,5), 0)
        return cv2.Canny(blur, 50, 150)
    
    def mask_roi(img):
        mask = np.zeros_like(img)

        vert = np.array(
            [
            [
                (int(0), int(mask.shape[0]*4/7)),
                (int(mask.shape[1]*2/12), int(mask.shape[0]*2/5)),
                (int(mask.shape[1]*10/12), int(mask.shape[0]*2/5)),
                (int(mask.shape[1]), int(mask.shape[0]*4/7)),
                (int(mask.shape[1]), mask.shape[0]),
                (int(0), mask.shape[0])
            ]
            ]
        )
        cv2.fillPoly(mask, vert, 255)
        masked = cv2.bitwise_and(img, mask)
        return masked
   
    def hough_lines(img):
        return cv2.HoughLinesP(img, RHO, THETA, MIN_VOTES, np.array([]), minLineLength=MIN_LINE_LEN, maxLineGap=MAX_LINE_GAP)
    

    def formulate_lanes(lines, img):
        negative_slopes = []
        positive_slopes = []

        negative_intercepts = []
        positive_intercepts = []

        y_min = img.shape[0]
        y_max = img.shape[0]

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2-y1) / (x2-x1)
                intercept = y2 - slope*x2

            y_min = min(y_min, y1, y2)

            if slope > 0.0:
                positive_slopes.append(slope)
                positive_intercepts.append(intercept)

            elif slope < 0.0:
                negative_slopes.append(slope)
                negative_intercepts.append(intercept)
        
        positive_slope = np.mean(positive_slopes)
        negative_slope = np.mean(negative_slopes)

        positive_intercept = np.mean(positive_intercepts)
        negative_intercept = np.mean(negative_intercepts)

        pts = [
            [[0,0, 0,0]],
            [[0,0, 0,0]]
        ]

        # +ve
        if len(positive_slopes) > 0:
            x_max = (y_max - positive_intercept) / positive_slope
            x_min = (y_min - positive_intercept) / positive_slope
            pts[0][0] = [x_min, y_min, x_max, y_max]
        
        if len(negative_slopes) > 0:
            x_max = (y_max - negative_intercept) / negative_slope
            x_min = (y_min - negative_intercept) / negative_slope
            pts[1][0] = [x_min, y_min, x_max, y_max]
       

        we = np.array(pts, dtype=np.float32)
        
        new = np.where(np.isnan(we),0,we)
        
        return np.array(new, dtype=np.int32)
        
    

    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        if (min(line1[0][0] , line1[1][0]) <= x <= max(line1[0][0] , line1[1][0])) and (min(line1[0][1] , line1[1][1]) <= y <= max(line1[0][1] , line1[1][1])):
            if (min(line2[0][0] , line2[1][0]) <= x <= max(line2[0][0] , line2[1][0])) and (min(line2[0][1] , line2[1][1]) <= y <= max(line2[0][1] , line2[1][1])):
                return (x, y)
        return None
    
    def offset_in_m(lane1,lane2):
        for i in range(250,0,-10):
            line = [(0,i),(320,i)]
            inter1 = line_intersection(lane1, line)
            inter2 = line_intersection(lane2, line)
            if inter1 == None or inter2 == None:
                continue
            dist = np.sqrt(np.square(inter1[0]-inter2[0])+np.square(inter1[1]-inter2[1]))
            mid_point = ((inter1[0] + inter2[0])/2 , (inter1[1] + inter2[1])/2 )
            camera_positon = (160,i)
            offset = np.sqrt(np.square(mid_point[0]-camera_positon[0])+np.square(mid_point[1]-camera_positon[1]))
            if mid_point[0] < 160:
                offset *= -1
            offset_in_m = (3.6*offset)/dist
            return offset_in_m,mid_point
        return None
    
    w = np.array(mask*255, dtype=np.uint8)
    w = np.squeeze(w, (0,3))
    w = np.where(w>MASK_THRESHOLD,np.ones_like(w)*255,np.zeros_like(w))
    edges = find_edges(w)
    masked = mask_roi(edges)
    lines = hough_lines(masked)
    lanes = formulate_lanes(lines, masked)  
    lane1 = [(lanes[0,0,0],lanes[0,0,1]),(lanes[0,0,2],lanes[0,0,3])]
    lane2 = [(lanes[1,0,0],lanes[1,0,1]),(lanes[1,0,2],lanes[1,0,3])]
    
    return offset_in_m(lane1,lane2)


def get_anchors(
    image_shape,
    strides=(8, 16, 32),
    base_anchors=(0.5, 0.5),
):
    base_anchors = np.array(base_anchors, dtype=np.float32)

    all_anchors = []
    all_strides = []
    for stride in strides:
        hh_centers = np.arange(0, image_shape[0], stride)
        ww_centers = np.arange(0, image_shape[1], stride)
        ww_grid, hh_grid = np.meshgrid(ww_centers, hh_centers)
        grid = np.array(
            np.reshape(np.stack([hh_grid, ww_grid], 2), [-1, 1, 2]),
            np.float32,
        )
        anchors = (
            np.expand_dims(
                base_anchors * np.array([stride, stride], np.float32), 0
            )
            + grid
        )
        anchors = np.reshape(anchors, [-1, 2])
        all_anchors.append(anchors)
        all_strides.append(np.repeat(stride, anchors.shape[0]))

    all_anchors = np.array(np.concatenate(all_anchors, axis=0), np.float32)
    all_strides = np.array(np.concatenate(all_strides, axis=0), np.float32)

    all_anchors = all_anchors / all_strides[:, None]

    all_anchors = np.concatenate(
        [all_anchors[:, 1, None], all_anchors[:, 0, None]], axis=-1
    )
    return all_anchors, all_strides


def dist2bbox(distance, anchor_points):
    left_top, right_bottom = np.split(distance, 2, axis=-1)
    x1y1 = anchor_points - left_top
    x2y2 = anchor_points + right_bottom
    return np.concatenate((x1y1, x2y2), axis=-1)


def box_area(xmin, ymin, xmax, ymax):
    w = xmax - xmin
    h = ymax - ymin
    
    return w*h


def iou(box1, box2):
    box1_xmin = min(box1[0], box1[2])
    box1_xmax = max(box1[0], box1[2])
    box1_ymin = min(box1[1], box1[3])
    box1_ymax = max(box1[1], box1[3])
    
    box2_xmin = min(box2[0], box2[2])
    box2_xmax = max(box2[0], box2[2])
    box2_ymin = min(box2[1], box2[3])
    box2_ymax = max(box2[1], box2[3])
    
    inter_xmin = max(box1_xmin, box2_xmin)
    inter_xmax = min(box1_xmax, box2_xmax)
    inter_ymin = max(box1_ymin, box2_ymin)
    inter_ymax = min(box1_ymax, box2_ymax)
    
    box1_area = box_area(box1_xmin, box1_ymin, box1_xmax, box1_ymax)
    box2_area = box_area(box2_xmin, box2_ymin, box2_xmax, box2_ymax)
    inter_area = box_area(inter_xmin, inter_ymin, inter_xmax, inter_ymax)
    
    return inter_area/(box1_area+box2_area-inter_area)


def non_max_suppression_padded(
    box_prediction,
    confidence_prediction,
    class_prediction,
    max_output_size,
    iou_threshold,
    score_threshold
)-> tuple:
    
    confidence_prediction = confidence_prediction[0, :]
    class_prediction = np.argmax(class_prediction, axis=-1)[0, :]
    indices = np.arange(confidence_prediction.shape[0]).tolist()
    indices = sorted(indices, key=lambda i: confidence_prediction[i], reverse=True)
    idx = np.zeros(shape=(max_output_size,), dtype=np.int32)
    valid_det = 0
    
    for i in indices:
        if confidence_prediction[i] < score_threshold:
            break
        
        found = False
        for j in range(valid_det):
            index = idx[j]
            if class_prediction[i] == class_prediction[index]:
                if iou(box_prediction[i], box_prediction[index]) > iou_threshold:
                    found = True
                    break
        
        if found == False:
            idx[valid_det] = i
            valid_det += 1
            if valid_det == max_output_size:
                break
        
    return idx, valid_det


def non_max_suppression(inputs, max_detections, iou_threshold, confidence_threshold):
    box_prediction, class_prediction, dist_prediction = inputs['boxes'], inputs['classes'], inputs['distances']

    confidence_prediction = np.max(class_prediction, axis=-1)

    idx, valid_det = non_max_suppression_padded(
        box_prediction,
        confidence_prediction,
        class_prediction,
        max_output_size=max_detections,
        iou_threshold=iou_threshold,
        score_threshold=confidence_threshold
    )
    
    box_prediction = np.take_along_axis(
        box_prediction, np.expand_dims(idx, axis=-1), axis=0
    )
    box_prediction = np.reshape(
        box_prediction, (-1, max_detections, 4)
    )
    
    confidence_prediction = np.take_along_axis(
        confidence_prediction[0, :], idx, axis=0
    )
    
    class_prediction = np.take_along_axis(
        np.argmax(class_prediction, axis=-1)[0, :], idx, axis=0
    )
    
    dist_prediction = np.take_along_axis(
        dist_prediction[0, :, 0], idx, axis=0
    )

    bounding_boxes = {
        "boxes": box_prediction,
        "confidence": np.expand_dims(confidence_prediction, axis=0),
        "classes": np.expand_dims(class_prediction, axis=0),
        "distances": np.expand_dims(dist_prediction, axis=0),
        "num_detections": np.array([valid_det], dtype=np.int32),
    }

    return bounding_boxes


def softmax(x, axis=-1):
    return(np.exp(x)/np.exp(x).sum(axis=axis, keepdims=True))


class Decoder:
    def __init__(self, max_detections=100, iou_threshold=0.5, confidence_threshold=0.5):
        self.max_detections = max_detections
        self.iou_threshold = iou_threshold
        self.confidence_threshold = confidence_threshold
    
    def __call__(self, inputs):
        preds, images = inputs['preds'], inputs['images']

        boxes = preds['boxes']
        scores = preds['classes']
        distances = preds['distances']
        
        boxes = np.reshape(boxes, [-1, 4, 16])
        boxes = softmax(boxes) * np.arange(16, dtype=np.float32)
        boxes = np.sum(boxes, axis=-1)
        
        anchor_points, stride_tensor = get_anchors(image_shape=images.shape[1:])
        stride_tensor = np.expand_dims(stride_tensor, axis=-1)
        box_preds = dist2bbox(boxes, anchor_points) * stride_tensor
        
        return non_max_suppression({
                'boxes': box_preds,
                'classes': scores,
                'distances': distances
            },
            self.max_detections, 
            self.iou_threshold, 
            self.confidence_threshold
        )

def main():
    global t
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--mode',
        required=True,
        help="run mode ['image', 'video', 'camera']",
        type=str
    )
    parser.add_argument(
        '--input',
        help='input to the model',
        required=False
    )
    parser.add_argument(
        '--preview',
        help='y/n or path to a folder to save output to it.',
        type=str,
        required=False,
        default='n'
    )
    args = parser.parse_args()
    dist_yolo = ort.InferenceSession("dist-yolo.onnx")
    lane_model = ort.InferenceSession("model.onnx")
    decoder = Decoder(max_detections=MAX_DETECTIONS, iou_threshold=IOU_THRESHOLD, confidence_threshold=CONFIDENCE_THRESHOLD)
    
    print(f'took {time.time()-t} seconds to initialize the model.')
    
    
    if args.mode == 'image':
        t = time.time()
        
        if not os.path.exists(args.input):
            raise ValueError('input file path doesn\'t exist.')
        input_data = np.expand_dims(
            np.array(
                cv2.cvtColor(
                    cv2.resize(
                        cv2.imread(args.input),
                        (640, 640)
                    ),
                    cv2.COLOR_BGR2RGB
                ),
                dtype=np.float32
            ),
            axis=0
        ) 
        results_ort = dist_yolo.run(["boxes", "classes", "distances"], {"args_0": input_data})
        
        lane_input = np.expand_dims(
            np.array(
                cv2.cvtColor(
                    cv2.resize(
                        cv2.imread(args.input),
                        (224, 224)
                    ),
                    cv2.COLOR_BGR2RGB
                ),
                dtype=np.uint8
            ),
            axis=0
        )
         
        lane_out = lane_model.run(['segments'], {'args_0': lane_input})[0]
        
        lane_post = lane_postprocessing(lane_out)
        
        if lane_post != None:
            offset, lane_mid = lane_post
            print('lane offset:\t', offset)
        
        pred = decoder({
            'preds': {
                "boxes": results_ort[0], 
                "classes": results_ort[1], 
                "distances": results_ort[2]
                },
            'images': input_data
        })
        
        angles = calculate_horizontal_angle(pred)
        
        if args.preview != 'n':
            frame = cv2.resize(cv2.imread(args.input), (640, 640))
            
            boxes = []
            classes = []
            dist = []
            conf = []

            for i in range(pred['num_detections'][0]):
                boxes.append(list(map(int,pred["boxes"][0][i].tolist())))
                classes.append(pred["classes"][0][i].tolist())
                dist.append(pred['distances'][0][i].tolist())
                conf.append(pred["confidence"][0][i].tolist())

            for i in range(pred['num_detections'][0]):
                frame = cv2.putText(
                    cv2.rectangle(
                        frame,
                        (boxes[i][0],boxes[i][1]),
                        (boxes[i][2],boxes[i][3]),
                        (0,255,0),
                        2
                    ),
                    str(classes_names[classes[i]])+"|"+str(round(dist[i], 2))+"|"+str(round(conf[i], 3)),
                    (boxes[i][0],boxes[i][1]-3),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2
                )
                
                bbox_center = ((boxes[i][2]+boxes[i][0])//2,(boxes[i][3]+boxes[i][1])//2)
                cv2.line(frame, (320, 640), bbox_center, (0,255,0), 1)
                midpoint = ((320+bbox_center[0])//2,(640+bbox_center[1])//2)
                offset_x = 10
                offset_y = 20
                txt_position = (midpoint[0]+offset_x,midpoint[1]+offset_y)     
                cv2.putText(
                    frame,
                    f'{str(round(angles[i]))}deg',
                    txt_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1
                )   
            
            if lane_post != None:
                lane_ref = np.array([320, 585], dtype=np.int32)
                lane_mid = np.array([lane_mid[0]*2, 585], dtype=np.int32)
                
                frame = cv2.circle(frame, lane_ref, 5, (255, 0, 0), -1)
                frame = cv2.circle(frame, lane_mid, 5, (255, 0, 0), -1)
                frame = cv2.line(frame, lane_mid, lane_ref, (255, 0, 0), 3)
                
                if offset < 0:
                    frame = cv2.putText(
                        frame,
                        "offset: "+str(round(offset, 2))+"m",
                        (lane_mid[0]+2,lane_mid[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2
                    )
                else:
                    frame = cv2.putText(
                        frame,
                        "offset: "+str(round(offset, 2))+"m",
                        (lane_ref[0]+2,lane_ref[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2
                    )
            
            if args.preview == 'y':
                cv2.imshow('test', frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                if not os.path.exists(args.preview):
                    raise ValueError('output folder path doesn\'t exist')
                cv2.imwrite(args.preview+'output.png', frame)
        
        print(pred, '\nangles:\t', angles)
        print(time.time()-t, 'sec\n\n')
    
    
    elif args.mode == 'video':
        if not os.path.exists(args.input):
            raise ValueError('input file path doesn\'t exist.')
        if args.preview not in ['y', 'n']:
            if not os.path.exists(args.preview):
                raise ValueError('output folder path doesn\'t exist')
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            output_video = cv2.VideoWriter(
                args.preview+'output.mp4',
                fourcc,
                30,
                (640, 640)
            )
        video_capture = cv2.VideoCapture(args.input)
        num_frame = 0
        t = time.time()
        while True:
            ret, frame = video_capture.read()
            if not ret:
                break
            frame = cv2.resize(
                frame,
                (640, 640)
            )
            input_data = np.expand_dims(
                np.array(
                    cv2.cvtColor(
                        frame,
                        cv2.COLOR_BGR2RGB
                    ),
                    dtype=np.float32
                ),
                axis=0
            ) 
            results_ort = dist_yolo.run(["boxes", "classes", "distances"], {"args_0": input_data})
            
            lane_input = np.expand_dims(
                np.array(
                    cv2.cvtColor(
                        cv2.resize(
                            frame,
                            (224, 224)
                        ),
                        cv2.COLOR_BGR2RGB
                    ),
                    dtype=np.uint8
                ),
                axis=0
            )
            
            lane_out = lane_model.run(['segments'], {'args_0': lane_input})[0]
            
            lane_post = lane_postprocessing(lane_out)
            
            if lane_post != None:
                offset, lane_mid = lane_post
                print('lane offset:\t', offset)
            
            pred = decoder({
                'preds': {
                    "boxes": results_ort[0], 
                    "classes": results_ort[1], 
                    "distances": results_ort[2]
                    },
                'images': input_data
            })
            
            angles = calculate_horizontal_angle(pred)
            
            if args.preview != 'n':
                boxes = []
                classes = []
                dist = []
                conf = []

                for i in range(pred['num_detections'][0]):
                    boxes.append(list(map(int,pred["boxes"][0][i].tolist())))
                    classes.append(pred["classes"][0][i].tolist())
                    dist.append(pred['distances'][0][i].tolist())
                    conf.append(pred["confidence"][0][i].tolist())

                for i in range(pred['num_detections'][0]):
                    frame = cv2.putText(
                        cv2.rectangle(
                            frame,
                            (boxes[i][0],boxes[i][1]),
                            (boxes[i][2],boxes[i][3]),
                            (0,255,0),
                            2
                        ),
                        str(classes_names[classes[i]])+"|"+str(round(dist[i], 2))+"|"+str(round(conf[i], 3)),
                        (boxes[i][0],boxes[i][1]-3),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,255,0),
                        2
                    )
                    
                    bbox_center = ((boxes[i][2]+boxes[i][0])//2,(boxes[i][3]+boxes[i][1])//2)
                    cv2.line(frame, (320, 640), bbox_center, (0,255,0), 1)
                    midpoint = ((320+bbox_center[0])//2,(640+bbox_center[1])//2)
                    offset_x = 10
                    offset_y = 20
                    txt_position = (midpoint[0]+offset_x,midpoint[1]+offset_y)     
                    cv2.putText(
                        frame,
                        f'{str(round(angles[i]))}deg',
                        txt_position,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        1
                    )   
                    
                if lane_post != None:
                    lane_ref = np.array([320, 585], dtype=np.int32)
                    lane_mid = np.array([lane_mid[0]*2, 585], dtype=np.int32)
                    
                    frame = cv2.circle(frame, lane_ref, 5, (255, 0, 0), -1)
                    frame = cv2.circle(frame, lane_mid, 5, (255, 0, 0), -1)
                    frame = cv2.line(frame, lane_mid, lane_ref, (255, 0, 0), 3)
                    
                    if offset < 0:
                        frame = cv2.putText(
                            frame,
                            "offset: "+str(round(offset, 2))+"m",
                            (lane_mid[0]+2,lane_mid[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 0, 0),
                            2
                        )
                    else:
                        frame = cv2.putText(
                            frame,
                            "offset: "+str(round(offset, 2))+"m",
                            (lane_ref[0]+2,lane_ref[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 0, 0),
                            2
                        )

                if args.preview == 'y':
                    cv2.imshow('test', frame)
                else:
                    output_video.write(frame)
            
            print(pred, '\nangles:\t', angles, '\n')
            num_frame += 1
            print('Frame:\t'+str(num_frame), ',\t\t', time.time()-t, 'sec\n\n')
            t = time.time()
        video_capture.release()
        if args.preview not in ['y', 'n']:
            output_video.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    
    # elif args.mode == 'camera':
    #     if args.preview not in ['y', 'n']:
    #         if not os.path.exists(args.preview):
    #             raise ValueError('output folder path doesn\'t exist')
    #         fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #         output_video = cv2.VideoWriter(
    #             args.preview+'output.mp4',
    #             fourcc,
    #             30,
    #             (640, 640)
    #         )
    #     with picamera2.Picamera2() as camera:
    #         config = camera.create_preview_configuration({'format': 'RGB888'})
    #         camera.configure(config)
    #         camera.start()
    #         num_frame = 0
    #         t = time.time()
    #         while True:
    #             frame = camera.capture_array()
    #             frame = cv2.resize(
    #                 frame,
    #                 (640, 640)
    #             )
    #             input_data = np.expand_dims(
    #                 np.array(
    #                     frame,
    #                     dtype=np.float32
    #                 ),
    #                 axis=0
    #             ) 
    #             results_ort = dist_yolo.run(["boxes", "classes", "distances"], {"args_0": input_data})
        
    #             lane_input = np.expand_dims(
    #                 np.array(
    #                     cv2.cvtColor(
    #                         cv2.resize(
    #                             frame,
    #                             (320, 256)
    #                         ),
    #                         cv2.COLOR_BGR2RGB
    #                     ),
    #                     dtype=np.float32
    #                 ),
    #                 axis=0
    #             )
                
    #             lane_out = lane_model.run(['conv2d_3'], {'args_0': lane_input})[0]
                
    #             lane_post = lane_postprocessing(lane_out)
            
    #             if lane_post != None:
    #                 offset, lane_mid = lane_post
    #                 print('lane offset:\t', offset)
        
    #             pred = decoder({
    #                 'preds': {
    #                     "boxes": results_ort[0], 
    #                     "classes": results_ort[1], 
    #                     "distances": results_ort[2]
    #                     },
    #                 'images': input_data
    #             })
        
    #             angles = calculate_horizontal_angle(pred)
                
    #             if args.preview != 'n':
    #                 boxes = []
    #                 classes = []
    #                 dist = []
    #                 conf = []

    #                 for i in range(pred['num_detections'][0]):
    #                     boxes.append(list(map(int,pred["boxes"][0][i].tolist())))
    #                     classes.append(pred["classes"][0][i].tolist())
    #                     dist.append(pred['distances'][0][i].tolist())
    #                     conf.append(pred["confidence"][0][i].tolist())

    #                 for i in range(pred['num_detections'][0]):
    #                     frame = cv2.putText(
    #                         cv2.rectangle(
    #                             frame,
    #                             (boxes[i][0],boxes[i][1]),
    #                             (boxes[i][2],boxes[i][3]),
    #                             (0,255,0),
    #                             2
    #                         ),
    #                         str(classes_names[classes[i]])+"|"+str(round(dist[i], 2))+"|"+str(round(conf[i], 3)),
    #                         (boxes[i][0],boxes[i][1]-3),
    #                         cv2.FONT_HERSHEY_SIMPLEX,
    #                         0.5,
    #                         (0,255,0),
    #                         2
    #                     )
        
    #                     bbox_center = ((boxes[i][2]+boxes[i][0])//2,(boxes[i][3]+boxes[i][1])//2)
    #                     cv2.line(frame, (320, 640), bbox_center, (0,255,0), 1)
    #                     midpoint = ((320+bbox_center[0])//2,(640+bbox_center[1])//2)
    #                     offset_x = 10
    #                     offset_y = 20
    #                     txt_position = (midpoint[0]+offset_x,midpoint[1]+offset_y)     
    #                     cv2.putText(
    #                         frame,
    #                         f'{str(round(angles[i]))}deg',
    #                         txt_position,
    #                         cv2.FONT_HERSHEY_SIMPLEX,
    #                         0.5,
    #                         (0, 0, 255),
    #                         1
    #                     )   
        
    #                 if lane_post != None:
    #                     lane_ref = np.array([320, 585], dtype=np.int32)
    #                     lane_mid = np.array([lane_mid[0]*2, 585], dtype=np.int32)
                        
    #                     frame = cv2.circle(frame, lane_ref, 5, (255, 0, 0), -1)
    #                     frame = cv2.circle(frame, lane_mid, 5, (255, 0, 0), -1)
    #                     frame = cv2.line(frame, lane_mid, lane_ref, (255, 0, 0), 3)
                        
    #                     if offset < 0:
    #                         frame = cv2.putText(
    #                             frame,
    #                             "offset: "+str(round(offset, 2))+"m",
    #                             (lane_mid[0]+2,lane_mid[1]-5),
    #                             cv2.FONT_HERSHEY_SIMPLEX,
    #                             0.5,
    #                             (255, 0, 0),
    #                             2
    #                         )
    #                     else:
    #                         frame = cv2.putText(
    #                             frame,
    #                             "offset: "+str(round(offset, 2))+"m",
    #                             (lane_ref[0]+2,lane_ref[1]-5),
    #                             cv2.FONT_HERSHEY_SIMPLEX,
    #                             0.5,
    #                             (255, 0, 0),
    #                             2
    #                         )

    #                 if args.preview == 'y':
    #                     cv2.imshow('test', frame)
    #                 else:
    #                     output_video.write(frame)
                
    #             print(pred, '\nangles:\t', angles, '\n')
    #             num_frame += 1
    #             print('Frame:\t'+str(num_frame), ',\t\t', time.time()-t, 'sec\n\n')
    #             t = time.time()
    #         camera.stop()
    #         if args.preview not in ['y', 'n']:
    #             output_video.release()
    #         cv2.waitKey(0)
    #         cv2.destroyAllWindows()
    
    else:
        raise ValueError('unexpected \'mode\' argument value')

if __name__ == '__main__':
    main()
