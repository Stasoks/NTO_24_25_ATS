import rospy
from gs_module import BoardLedController
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_navigation import NavigationManager
from rospy import sleep
import cv2
import numpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


rospy.init_node("flight_test_node") 


board = BoardManager() 
led = BoardLedController()
navigation = NavigationManager()


schet = 0

# coordinates = [ 
#     #[1.33, 3.7, 1.4],#start
#     [2.3, 3.5, 1.4],#1
#     [1.55, 4.5, 1.4],#2
#     [1.1, 5.3, 1.4],#3
#     [0.46, 3.5, 1.4],#4
#     [0.1, 3.5, 1.4],#5
#     [0.47, 0.9, 1.4],#6
#     [0.15, 1, 1.4],#7
#     [1.2, 0.14, 1.4],#8
#     [1.4, 0.05, 1.4],#9
#     [3, 1, 1.4],#10
#     [1.4, 1.25, 1.2]#финиш
# ]
landing_pad = [1.3355, 4.586, 1.1997, 97.03]
# coordinates = [
# [0.2336, 4.3025, 1.1949, 92.28 ],
# [0.2707, 3.56  , 1.1657, 52.73 ],
# [1.1976, 2.5386, 1.0803, 46.4  ],
# [0.4715, 1.1553, 0.6385, 133.41],
# [0.197 , 1.2513, 1.07  , 156.79],
# # [0.0   , 0.0   , 0.0   , 107.22],
# [0.1852, 0.8087, 1.0073, 81.0  ],
# [0.6548, 0.4945, 1.0094, 59.23 ],
# [1.0358, 0.5085, 0.9622, 68.73 ],
# [2.4062, 2.5284, 1.1309, -49.4 ],
# [1.7421, 2.9941, 1.1241, -36.92],
# [2.3195, 3.6218, 1.1917, -62.76]]

coordinates = [
    [0.32, 3.97, 1.9, 82.6],
    [0.548, 0.7464, 1.9926, -49.4],
    [2.0, 2.693, 1.897, 26.7],
    [1.3355, 4.586, 1.1997, 97.03]
]

run = True 
position_number = 0 

def get_img():
    bridge = CvBridge()
    data = rospy.wait_for_message("pioneer_max_camera/image_raw",Image)
    frame = bridge.imgmsg_to_cv2(data,"bgr8")
    
    return frame

def load_model():
    net = cv2.dnn.readNet("yolov4-tiny-obj_best.weights","yolov4-tiny-obj-test.cfg")
    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(size=(416,416),scale = 1/255, swapRB=True)#416,416

    return model

def detect_model(model,frame):
    NMS_threshold = 0.5
    Conf_threshold = 0.5

    classes, scores, boxes = model.detect(frame,Conf_threshold,NMS_threshold)
    
    return zip(boxes, scores), classes

def give_coord(model, frame):
    detected, classes = detect_model(model, frame)
    # Список ЧС с вероятностью (confidense) больше 80
    ans = list(filter(lambda x: x[1] > 0.80, detected))
    #ans = sorted(detected, key = lambda x: x[1], reverse=True)[0] Один чс
    if len(classes):
        # Выводим кординаты ЧС и его тип
        schet +=1
        # Выводим кординаты ЧС и его тип


# model = load_model()
   
def callback(event): 
    global ap
    global run
    global coordinates
    global position_number


    event = event.data

    if event == CallbackEvent.ENGINES_STARTED:
        print("engine started - - - - - - - - -")
        ap.takeoff() 

    elif event == CallbackEvent.TAKEOFF_COMPLETE: 
        print("takeoff complite")
        
        position_number = 0 
        ap.goToLocalPoint(coordinates[position_number][0],
                           coordinates[position_number][1],
                           coordinates[position_number][2])
        ap.updateYaw(coordinates[position_number][3])
        # while event != CallbackEvent.POINT_REACHED:
        #     sleep(1)
        #     frame = get_img()
            
    
    elif event == CallbackEvent.POINT_REACHED:
        print("point {} reached".format(position_number))
        position_number += 1 
        if position_number < len(coordinates): 
            ap.goToLocalPoint(coordinates[position_number][0],
                               coordinates[position_number][1],
                               coordinates[position_number][2])
            
            give_coord(model, get_img())
            
   
        else:
            # ap.disarm()
            ap.landing() 


    elif event == CallbackEvent.COPTER_LANDED:
        print("finish programm") 
        run = False


ap = FlightController(callback) 

once = False 

while not rospy.is_shutdown() and run:
   if board.runStatus() and not once: 
       print("start programm")
       ap.preflight() 
       once = True
   pass


