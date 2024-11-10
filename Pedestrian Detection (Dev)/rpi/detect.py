from enum import Enum
import time
import os
import onnxruntime as ort
import numpy as np
import cv2 as cv
import argparse

time.sleep(5)
rpi = True if input('Import RPI Packages? [y/n]').lower().strip()[0] == 'y' else False
if rpi:
    import picamera2
    import sysv_ipc


# ALL DIRECTIONS ARE GIVEN RELATIVE TO POV

class Direction(Enum):
    LEFT = 'left'
    RIGHT = 'right'

class Color(Enum):
    RED = 'red'
    GREEN = 'green'
    YELLOW = 'yellow'

class Mode(Enum):
    DEF = ''
    IMAGE = ''
    VIDEO = ''
    CAMERA = ''

MOTOR_KEY_SEND = 12

def send_feedback_to_control(motor_queue, MsgType, message):
    message_str = f"{message}"
    feedback_message = message_str.encode()
    motor_queue.send(feedback_message)

def run_object_detection(net, image, score_th=0.3, iou_th=0.3):
    
    height, width = image.shape[:2]

    # Create a blob
    blob = cv.dnn.blobFromImage(image, scalefactor=1/255.0, size=(320, 320), swapRB=True, crop=False)
    net.setInput(blob)
    
    # Run forward pass
    outs = net.forward()

    # Initialization
    class_ids = []
    confidences = []
    boxes = []

    # For each detection from each output layer
    for i in range(outs.shape[0]):
        detection = outs[i]
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > score_th:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)
    
    # Perform non-maxima suppression to eliminate redundant overlapping boxes with lower confidences
    indices = cv.dnn.NMSBoxes(boxes, confidences, score_threshold=score_th, nms_threshold=iou_th)

    boxes = [boxes[i] for i in indices]
    class_ids = [class_ids[i] for i in indices]
    confidences = [confidences[i] for i in indices]
    
    rel_boxes = []
    for box in boxes:
        x, y, w, h = box
        xmin = x / width
        ymin = y / height
        xmax = (x + w) / width
        ymax = (y + h) / height
        rel_boxes.append([ymin, xmin, ymax, xmax])
    
    return rel_boxes, confidences, class_ids

def run_lane_detection(onnx_session, image):
    img_height, img_width = image.shape[:2]
    
    model_outputs = onnx_session.get_outputs()
    output_names = [model_outputs[i].name for i in range(len(model_outputs))]
    
    rgb_input_name = onnx_session.get_inputs()[0].name
    mask_input_name = onnx_session.get_inputs()[1].name
    input_size = onnx_session.get_inputs()[0].shape[2:]
    
    img = cv.resize(image,(input_size[1], input_size[0]))
    
    mean=[0.485, 0.456, 0.406]
    # std=[0.2, 0.3, 0.225]
    std=[0.229, 0.224, 0.225]
    
    img = ((img/ 255.0 - mean) / std)
    
    img = img.transpose(2, 0, 1)
    input_tensor = img[np.newaxis,:,:,:].astype(np.float32)

    mask_tensor = np.zeros((1, 1, input_size[0], input_size[1]), dtype=np.float32)
    
    # ratio = 0.5
    # mask_tensor[0,0, int(ratio*input_size[0]), int(ratio*input_size[1])] = 1
    
    outputs = onnx_session.run(output_names, {rgb_input_name: input_tensor, 
                                              mask_input_name: mask_tensor})
    
    pred_logits = outputs[0]
    pred_curves = outputs[1]

    # Filter good lanes based on the probability
    pred_logits_ex = np.exp(pred_logits - np.max(pred_logits))
    prob = pred_logits_ex / pred_logits_ex.sum(axis=-1).T
    good_detections = np.where(np.argmax(prob,axis=-1)==1)
    pred_logits = pred_logits[good_detections]
    pred_curves = pred_curves[good_detections]
    
    lanes = []
    for lane_data in pred_curves:
        bounds = lane_data[:2]
        k_2, f_2, m_2, n_1, b_2, b_3 = lane_data[2:]

        # Calculate the points for the lane
        # Note: the logspace is used for a visual effect, np.linspace would also work as in the original repository
        y_norm = bounds[0]+np.logspace(0,2, 50, base=1/10, endpoint=True)*(bounds[1]-bounds[0])
        x_norm = (k_2 / (y_norm - f_2) ** 2 + m_2 / (y_norm - f_2) + n_1 + b_2 * y_norm - b_3)
        lane_points = np.vstack((x_norm*img_width, y_norm*img_height)).astype(int)
        
        lanes.append(lane_points)    

    return lanes, good_detections[1].tolist()

def lane_angle(lanes, frame_height, num_points=15):
    
    if len(lanes) < 2:
        return 90.0, 'left', np.array([]), 90.0, 'right'
    
    lanes = list(map(lambda x: np.array([x[0], frame_height-x[1]]), lanes))
    
    slopes = []
    for lane in lanes:
        lane_slope = np.polyfit(lane[0, :num_points], lane[1, :num_points], 1)[0]
        slopes.append(lane_slope)
    
    i_left = np.argmin(slopes)
    i_right = np.argmax(slopes)
    
    lane_right = lanes[i_right]
    lane_left = lanes[i_left]
    
    lane_mid = (lane_right + lane_left) / 2
    
    mid_slope = np.polyfit(lane_mid[0, :num_points], lane_mid[1, :num_points], 1)[0]
    direction_slope = np.polyfit(lane_mid[0, num_points:], lane_mid[1, num_points:], 1)[0]
    
    offset = None
    if mid_slope > 0:
        offset = Direction.LEFT
    else:
        offset = Direction.RIGHT

    direction = None
    if direction_slope > 0:
        direction = Direction.RIGHT
    else:
        direction = Direction.LEFT
    
    direction_angle = np.degrees(np.arctan(np.abs(direction_slope)))
    
    angle = np.degrees(np.arctan(np.abs(mid_slope)))
    
    return angle, offset.value, np.array([lane_mid[0], frame_height-lane_mid[1]]), direction_angle, direction.value

def detections_distances_and_angles(boxes, classes, dims_file='./data/coco.dims', fov=55, aspect_ratio=0.7):
    class_dims = list(map(str.split, open(dims_file, 'r').readlines()))
    
    dists = []
    angles = []
    directions = []
    
    for box, category in zip(boxes, classes):
        box_area = abs(box[0]-box[2]) * abs(box[1]-box[3])
        
        x, y, h = tuple(map(int, class_dims[category]))
        
        if max(x, y, h) == h or min(x, y, h) == h:
            class_area = h * y
        elif abs(box[0]-box[2]) > abs(box[1]-box[3]):
            class_area = h * min(x, y)
        else:
            class_area = h * max(x, y)
        
        object_plane_area  = class_area / box_area
        object_plane_width = np.sqrt(object_plane_area/aspect_ratio)
        
        distance = object_plane_width / (2*np.tan(np.radians(fov)))
        
        box_center = (box[1]+box[3])/2, 1.0-(box[0]+box[2])/2
        
        slope = box_center[1] / (box_center[0] - 0.5)
        
        if slope < 0:
            direction = Direction.LEFT
        else:
            direction = Direction.RIGHT
        
        angle = np.degrees(np.arctan(np.abs(slope)))
        
        dists.append(distance)
        angles.append(angle)
        directions.append(direction.value)
    
    return dists, angles, directions

def traffic_light_color(image, boxes, conf, classes):
    
    # TODO: Add Hough Transform to detect circles
    
    lights = []
    confidences = []
    
    for box, c, category in zip(boxes, conf, classes):
        
        if category != 9:
            continue
        
        img = image[int(box[0]*image.shape[0]):int(box[2]*image.shape[0]), int(box[1]*image.shape[1]):int(box[3]*image.shape[1])]
        
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # color range
        lower_red1 = np.array([0,100,100])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([160,100,100])
        upper_red2 = np.array([180,255,255])
        lower_green = np.array([40,50,50])
        upper_green = np.array([90,255,255])
        # lower_yellow = np.array([15,100,100])
        # upper_yellow = np.array([35,255,255])
        lower_yellow = np.array([15,150,150])
        upper_yellow = np.array([35,255,255])
        lower_blue= np.array([78,158,124])
        upper_blue = np.array([138,255,255])
        
        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        maskr = cv.add(mask1, mask2)
        
        maskg = cv.inRange(hsv, lower_green, upper_green)
        # mask2 = cv.inRange(hsv, lower_blue, upper_blue)
        # maskg = cv.add(mask1, mask2)
        
        masky = cv.inRange(hsv, lower_yellow, upper_yellow)
        
        red = np.count_nonzero(np.squeeze(maskr))
        green = np.count_nonzero(np.squeeze(maskg))
        yellow = np.count_nonzero(np.squeeze(masky))
        
        light = None
        
        if red > green and red > yellow:
            light = Color.RED
        
        if green > red and green > yellow:
            light = Color.GREEN
        
        if yellow > red and yellow > green:
            light = Color.YELLOW
        
        if light is not None:
            lights.append(light.value)
            confidences.append(c)
    
    if len(lights) == 0:
        return None
    
    return lights[np.argmax(confidences)]

def visualise_lanes(input_img, lanes, lane_ids, angle, offset, direction_angle, direction):

    lane_colors = [
        (68,  65,  249),
        (44,  114, 243),
        (30,  150, 248),
        (74,  132, 249),
        (79,  199, 249),
        (109, 190, 144),
        (142, 144, 77), 
        (161, 125, 39),
        (100, 0, 20)
    ]
    
    # Write the detected line points in the image
    visualization_img = input_img.copy()
    
    frame_height, frame_width = visualization_img.shape[:2]

    # Draw a mask for the current lane
    right_lane = np.where(lane_ids==0)[0]
    left_lane = np.where(lane_ids==5)[0]

    if(len(left_lane) and len(right_lane)):
        lane_segment_img = visualization_img.copy()

        points = np.vstack((lanes[left_lane[0]].T,
                            np.flipud(lanes[right_lane[0]].T)))
        cv.fillConvexPoly(lane_segment_img, points, color =(0,191,255))
        visualization_img = cv.addWeighted(visualization_img, 0.7, lane_segment_img, 0.3, 0)
        
    for lane_num,lane_points in zip(lane_ids, lanes):
        for lane_point in lane_points.T:
            cv.circle(visualization_img, (int(lane_point[0]), int(lane_point[1])), 3, lane_colors[lane_num], -1)
    
    cv.putText(visualization_img, f'Lane Offset: {round(angle, 1)}deg {offset}, Lane Direction: {round(direction_angle, 1)}deg {direction}',
               (int(frame_width/4), frame_height - 20), cv.FONT_HERSHEY_SIMPLEX, frame_width/1500.0, (100, 30, 30), 2)
    
    return visualization_img

def visualize_detections(input_img, boxes, classes, conf, dists, angles, directions, light_color, names_file='./data/coco.names'):
    class_names = list(map(lambda x: x[:-1], open(names_file, 'r').readlines()))
    
    colors = [
        [173, 216, 230],  # Light Blue
        [255, 160, 122],  # Light Salmon
        [144, 238, 144],  # Light Green
        [255, 182, 193],  # Light Pink
        [255, 228, 196],  # Bisque
        [224, 255, 255],  # Light Cyan
        [255, 239, 213],  # Papaya Whip
        [175, 238, 238],  # Pale Turquoise
        [219, 112, 147],  # Pale Violet Red
        [255, 218, 185],  # Peach Puff
        [152, 251, 152],  # Pale Green
        [240, 230, 140],  # Khaki
        [238, 232, 170],  # Pale Goldenrod
        [240, 255, 240],  # Honeydew
        [230, 230, 250],  # Lavender
        [250, 250, 210],  # Light Goldenrod Yellow
        [245, 245, 245],  # White Smoke
        [255, 228, 225],  # Misty Rose
        [245, 222, 179],  # Wheat
        [255, 240, 245],  # Lavender Blush
        [245, 245, 220],  # Beige
        [255, 250, 240],  # Floral White
        [240, 255, 255],  # Azure
        [240, 248, 255],  # Alice Blue
        [248, 248, 255]   # Ghost White
    ]
    
    colors = iter(np.array(colors, dtype=np.int32)[np.random.permutation(25)].tolist())
    
    visualization_img = input_img.copy()
    
    frame_height, frame_width = visualization_img.shape[:2]
    
    for bbox, score, class_id, dist, angle, direction in zip(boxes, conf, classes, dists, angles, directions):
        y1, x1 = int(bbox[0] * frame_height), int(bbox[1] * frame_width)
        y2, x2 = int(bbox[2] * frame_height), int(bbox[3] * frame_width)
        
        if y1 > frame_height:
            y1 = frame_height
        if y1 < 0:
            y1 = 0
        
        if y2 > frame_height:
            y2 = frame_height
        if y2 < 0:
            y2 = 0
        
        if x1 > frame_width:
            x1 = frame_width
        if x1 < 0:
            x1 = 0
        
        if x2 > frame_width:
            x2 = frame_width
        if x2 < 0:
            x2 = 0
        
        color = next(colors)
        
        cv.rectangle(visualization_img, (x1, y1), (x2, y2), color, 2)
        
        cv.putText(visualization_img, f'{class_names[class_id]} : ' + '%.2f' % (score),
                    (x1, y1 - 24), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        cv.putText(visualization_img, 'd:%.1fcm, a:%.1fdeg, '%(dist, angle) + direction,
                    (x1, y1 - 8), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    cv.putText(visualization_img, f'Traffic Light Color: {light_color}', 
               (int(frame_width*3/8), 20), cv.FONT_HERSHEY_SIMPLEX, frame_width/1500.0, (155, 0, 255), 2)
    
    return visualization_img

if __name__ == '__main__':
     
    parser = argparse.ArgumentParser()
    
    parser.add_argument(
        '--mode',
        help="run mode ['image', 'video', 'camera']",
        required=False,
        type=str,
        default='def'
    )
    parser.add_argument(
        '--input',
        help='input to the model',
        required=False,
        type=str,
        default='./samples/img.jpg'
    )
    parser.add_argument(
        '--preview',
        help='y/n or path to a folder to save output to it.',
        required=False,
        type=str,
        default='y'
    )
    
    args = parser.parse_args()
    
    mode, input, preview = args.mode, args.input, args.preview
    
    mode = str.lower(mode)
    preview = str.lower(preview)
    
    if mode == 'def':
        if input[-3:] in ['png', 'jpg']:
            mode = 'image'
        elif input[-3:] in ['mkv', 'mp4']:
            mode = 'video'

    if mode == 'image' and input[-3:] not in ['png', 'jpg']:
        raise ValueError(f'image mode but recieved input {input}.')
    elif mode == 'video' and input[-3:] not in ['mkv', 'mp4']:
        raise ValueError(f'video mode but recieved input {input}.')
    
    
    print('\n\nALL DIRECTIONS ARE GIVEN RELATIVE TO POV\n\n')
    if rpi:
        motor_queue_send = sysv_ipc.MessageQueue(MOTOR_KEY_SEND, sysv_ipc.IPC_CREAT)
    
    detection_net = cv.dnn.readNetFromDarknet('./data/object_detection.cfg', './data/object_detection.weights')
    lane_session = ort.InferenceSession('./data/lane_detection.onnx')
    
    if mode == 'image':
        input = cv.imread(input)
        frame_height, frame_width = input.shape[:2]
        
        t = time.time()
        
        boxes, conf, classes = run_object_detection(detection_net, input)
        lanes, lane_ids = run_lane_detection(lane_session, input)
        
        t = time.time()-t
        # print('inference latency           :\t\t', t, 's')
        
        t = time.time()
        
        angle, offset, mid_lane, direction_angle, direction = lane_angle(lanes, frame_height)
        
        # print('Angle :\t', angle, '\t\tDirection :\t', direction)
        lanes.append(mid_lane)
        lane_ids.append(8)
        
        dists, angles, directions = detections_distances_and_angles(boxes, classes)
        
        light_color = traffic_light_color(input, boxes, conf, classes)
        
        if preview not in ['no', 'n']:
            out_image = visualise_lanes(input, lanes, lane_ids, angle, offset, direction_angle, direction)
            out_image = visualize_detections(out_image, boxes, classes, conf, dists, angles, directions, light_color)
        
        t = time.time()-t
        # print('postprocessing latency      :\t\t', t, 's')
        
        class_names = list(map(lambda x: x[:-1], open('./data/coco.names', 'r').readlines()))

        if light_color == 'red':
            recommendation = 'stop'
        elif light_color == 'yellow':
            recommendation = 'warn'
        else:
            recommendation = 'okay'
        
        message = f'{round(angle, 1)}/{offset[0]}/{round(direction_angle, 1)}/{direction[0]}&{len(boxes)}/{str(list(map(lambda x: class_names[x], classes)))[1:-1]}/{str(list(map(lambda x: round(x, 1), angles)))[1:-1]}/{str(list(map(lambda x: round(x, 1), dists)))[1:-1]}&None,{light_color}/{recommendation}'.replace('\'', '') # TODO: edit message string
        
        if rpi:
            send_feedback_to_control(motor_queue_send, 4, message)
        
        if preview in ['y', 'yes']:
            cv.imshow('detection', out_image)
            if cv.waitKey() & 0xFF == 27:
                cv.destroyAllWindows()
        
        elif preview not in ['n', 'no']:
            cv.imwrite('output.png', out_image)
    
    elif mode == 'video':
        
        if preview not in ['y', 'n', 'yes', 'no']:
            if not os.path.exists(preview):
                raise ValueError(f'preview value must bet either "y", "n", "yes", "no" or a path to an existing directory to write output in it. got "{preview}".')
            out = cv.VideoWriter(os.path.join(preview, 'output.mp4'),cv.VideoWriter_fourcc('m','p','4','v'), 25, (1280,720))
        cap = cv.VideoCapture(input)
        times = []
        frame_count = 0
        
        while True:
            start_time = time.time()
            
            ret, frame = cap.read()
            
            if not ret:
                break
            
            frame_height, frame_width = frame.shape[:2]
            frame_count += 1
            
            t = time.time()
            
            boxes, conf, classes = run_object_detection(detection_net, frame)
            lanes, lane_ids = run_lane_detection(lane_session, frame)
            
            t = time.time()-t
            # print('inference latency           :\t\t', t, 's')
            
            t = time.time()
            
            angle, offset, mid_lane, direction_angle, direction = lane_angle(lanes, frame_height)
            
            # print('Angle :\t', angle, '\t\tDirection :\t', direction)
            lanes.append(mid_lane)
            lane_ids.append(8)
            
            dists, angles, directions = detections_distances_and_angles(boxes, classes)

            light_color = traffic_light_color(frame, boxes, conf, classes)
            
            if preview not in ['no', 'n']:
                out_image = visualise_lanes(frame, lanes, lane_ids, angle, offset, direction_angle, direction)
                out_image = visualize_detections(out_image, boxes, classes, conf, dists, angles, directions, light_color)
            
            t = time.time()-t
            # print('postprocessing latency      :\t\t', t, 's')
            
            end_time = time.time()
            
            elapsed_time = end_time - start_time
            
            times.append(elapsed_time)
            
            print(f'frame {frame_count} took ' + '%.5f'%(elapsed_time) + 's')
            
            class_names = list(map(lambda x: x[:-1], open('./data/coco.names', 'r').readlines()))
            
            if light_color == 'red':
                recommendation = 'stop'
            elif light_color == 'yellow':
                recommendation = 'warn'
            else:
                recommendation = 'okay'
            
            message = f'{round(angle, 1)}/{offset[0]}/{round(direction_angle, 1)}/{direction[0]}&{len(boxes)}/{str(list(map(lambda x: class_names[x], classes)))[1:-1]}/{str(list(map(lambda x: round(x, 1), angles)))[1:-1]}/{str(list(map(lambda x: round(x, 1), dists)))[1:-1]}&None,{light_color}/{recommendation}'.replace('\'', '') # TODO: edit message string
            
            if rpi:
                send_feedback_to_control(motor_queue_send, 4, message)
            
            print(message)
            
            if preview in ['y', 'yes']:
                cv.imshow('detection', out_image)
                
                if cv.waitKey(10) & 0xFF == 27:
                    cv.destroyAllWindows()
                    break
            
            elif preview not in ['n', 'no']:
                out.write(cv.resize(out_image, (1280, 720)))
        
        if preview not in ['y', 'n', 'yes', 'no']:
            out.release()
        
        cap.release()
        
        average_latency = np.average(times)
        
        print(f'Average Frame Delay :   '+'%.5f'%(average_latency))
        print(f'FPS :                   '+'%.5f'%(1/average_latency))
    
    elif mode == 'camera':
        
        if not rpi:
            raise ValueError('camera mode is only available on Raspberry Pi!')
        
        if preview not in ['y', 'n', 'yes', 'no']:
            if not os.path.exists(preview):
                raise ValueError(f'preview value must bet either "y", "n", "yes", "no" or a path to an existing directory to write output in it. got "{preview}".')
            out = cv.VideoWriter(os.path.join(preview, 'output.mp4'),cv.VideoWriter_fourcc('m','p','4','v'), 25, (1280,720))
        
        times = []
        
        with picamera2.Picamera2() as camera:
            config = camera.create_preview_configuration({'format': 'RGB888'})
            camera.configure(config)
            camera.start()
            
            frame_count = 0
            
            while True:
                start_time = time.time()
                
                frame = camera.capture_array()
                
                frame_height, frame_width = frame.shape[:2] # TODO: to be edited.
                frame_count += 1
                
                t = time.time()
            
                boxes, conf, classes = run_object_detection(detection_net, frame)
                lanes, lane_ids = run_lane_detection(lane_session, frame)
                
                t = time.time()-t
                # print('inference latency           :\t\t', t, 's')
                
                t = time.time()
                
                angle, offset, mid_lane, direction_angle, direction = lane_angle(lanes, frame_height)
                
                # print('Angle :\t', angle, '\t\tDirection :\t', direction)
                lanes.append(mid_lane)
                lane_ids.append(8)
                
                dists, angles, directions = detections_distances_and_angles(boxes, classes)
                
                light_color = traffic_light_color(frame, boxes, conf, classes)
                
                if preview not in ['no', 'n']:
                    out_image = visualise_lanes(frame, lanes, lane_ids, angle, offset, direction_angle, direction)
                    out_image = visualize_detections(out_image, boxes, classes, conf, dists, angles, directions, light_color)
                
                t = time.time()-t
                # print('postprocessing latency      :\t\t', t, 's')
                
                end_time = time.time()
                
                elapsed_time = end_time - start_time
                
                times.append(elapsed_time)
                
                print(f'frame {frame_count} took ' + '%.5f'%(elapsed_time) + 's')
                
                class_names = list(map(lambda x: x[:-1], open('./data/coco.names', 'r').readlines()))
                
                if light_color == 'red':
                    recommendation = 'stop'
                elif light_color == 'yellow':
                    recommendation = 'warn'
                else:
                    recommendation = 'okay'
                
                message = f'{round(angle, 1)}/{offset[0]}/{round(direction_angle, 1)}/{direction[0]}&{len(boxes)}/{str(list(map(lambda x: class_names[x], classes)))[1:-1]}/{str(list(map(lambda x: round(x, 1), angles)))[1:-1]}/{str(list(map(lambda x: round(x, 1), dists)))[1:-1]}&None,{light_color}/{recommendation}'.replace('\'', '') # TODO: edit message string
                
                send_feedback_to_control(motor_queue_send, 4, message)
                
                if preview in ['y', 'yes']:
                    cv.imshow('detection', out_image)
                    
                    if cv.waitKey(10) & 0xFF == 27:
                        cv.destroyAllWindows()
                        break
                elif preview not in ['n', 'no']:
                    out.write(cv.resize(out_image, (1280, 720)))
            
        if preview not in ['y', 'n', 'yes', 'no']:
            out.release()
        
        average_latency = np.average(times)
        
        print(f'Average Frame Delay :   '+'%.5f'%(average_latency))
        print(f'FPS :                   '+'%.5f'%(1/average_latency))

    else:
        raise ValueError(f'Mode not implemented. Mode is expected to be either "image" or "video" or "camera". got {mode}')
