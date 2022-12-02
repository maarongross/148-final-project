import cv2 as cv2 #importing the library
import numpy as np
import matplotlib.pyplot as plt
import depthai as dai
import math
import time
from simple_pid import PID

def show_image(name,img): #function for displaying the image
    cv2.imshow(name,img)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()

def find_canny(img,thresh_low,thresh_high): #function for implementing the canny
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # show_image('gray',img_gray)
    img_blur = cv2.GaussianBlur(img_gray,(5,5),0)
    # show_image('blur',img_blur)
    img_canny = cv2.Canny(img_blur,thresh_low,thresh_high)
    # show_image('Canny',img_canny)
    return img_canny

def region_of_interest(image): #function for extracting region of interest
    #bounds in (x,y) format
    # bounds = np.array([[[0,250],[0,200],[150,100],[500,100],[650,200],[650,250]]],dtype=np.int32)
    
    bounds = np.array([[[0,image.shape[0]],[0,2*image.shape[0]/3],[1920,2*image.shape[0]/3],[1920,image.shape[0]]]],dtype=np.int32)
    mask=np.zeros_like(image)
    cv2.fillPoly(mask,bounds,[255,255,255])
    # show_image('inputmask',mask)
    masked_image = cv2.bitwise_and(image,mask)
    # show_image('mask',masked_image) 
    return masked_image


def draw_lines(img,lines): #function for drawing lines on black mask
    mask_lines=np.zeros_like(img)
    for points in lines:
        x1,y1,x2,y2 = points[0]
        cv2.line(mask_lines,(x1,y1),(x2,y2),[0,0,255],2)

    return mask_lines

def get_coordinates(img,line_parameters): #functions for getting final coordinates
    slope=line_parameters[0]
    intercept = line_parameters[1]
    #y1 =300
    #y2 = 120
    y1=img.shape[0]
    y2 = 0.6*img.shape[0]
    if slope == 0:
        return[0,int(y1),0,int(y2)]
    x1= int((y1-intercept)/slope)
    x2 = int((y2-intercept)/slope)
    return [x1,int(y1),x2,int(y2)]

def compute_average_lines(img,lines):
    left_lane_lines=[]
    right_lane_lines=[]
    left_weights=[]
    right_weights=[]
    #print("line-count", len(lines))
    for points in lines:
        x1,y1,x2,y2 = points[0]
        if x2==x1:
            continue     
        parameters = np.polyfit((x1,x2),(y1,y2),1) #implementing polyfit to identify slope and intercept
        slope,intercept = parameters     
        
        length = np.sqrt((y2-y1)**2+(x2-x1)**2)
        if abs(slope) < 0.0001:
            slope = 0.0001
        if slope <0:
            left_lane_lines.append([slope,intercept])
            left_weights.append(length)         
        else:
            right_lane_lines.append([slope,intercept])
            right_weights.append(length)
        #print("slope: ", slope, "\nintercept: ", intercept)
    
    #print("left_lane: ", left_lane_lines, "\nright_lane: ", right_lane_lines)    

    #print("length: ", len(left_lane_lines))

    #Computing average slope and intercept
    if len(left_lane_lines) < 1: # check if there is a left lane or not
        left_average_line = [0,0]
        print("Left lane not detected")
    else:
        left_average_line = np.average(left_lane_lines,axis=0)
        print("Left lane detected")
    
    if len(right_lane_lines) < 1: # same as left lanes, but for right lanes
        right_average_line = [0,0]
        print("Right lane not detected")
    else:
        right_average_line = np.average(right_lane_lines,axis=0)
        print("Right lane detected")
    #print("left_average: ", left_average_line, "\nright_average: ", right_average_line)
    
    #print("Averages:",left_average_line,right_average_line)
    #Computing weigthed sum
    # if len(left_weights)>0:
    #     left_average_line = np.dot(left_weights,left_lane_lines)/np.sum(left_weights)
    # if len(right_weights)>0:
    #     right_average_line = np.dot(right_weights,right_lane_lines)/np.sum(right_weights)
    left_fit_points = get_coordinates(img,left_average_line)
    right_fit_points = get_coordinates(img,right_average_line) 
    #print("Fit points:",left_fit_points,right_fit_points)
    return [[left_fit_points],[right_fit_points]] #returning the final coordinates

class VESC:
    ''' 
    VESC Motor controler using pyvesc
    This is used for most electric scateboards.
    
    inputs: serial_port---- port used communicate with vesc. for linux should be something like /dev/ttyACM1
    has_sensor=False------- default value from pyvesc
    start_heartbeat=True----default value from pyvesc (I believe this sets up a heartbeat and kills speed if lost)
    baudrate=115200--------- baudrate used for communication with VESC
    timeout=0.05-------------time it will try before giving up on establishing connection
    
    percent=.2--------------max percentage of the dutycycle that the motor will be set to
    outputs: none
    
    uses the pyvesc library to open communication with the VESC and sets the servo to the angle (0-1) and the duty_cycle(speed of the car) to the throttle (mapped so that percentage will be max/min speed)
    
    Note that this depends on pyvesc, but using pip install pyvesc will create a pyvesc file that
    can only set the speed, but not set the servo angle. 
    
    Instead please use:
    pip install git+https://github.com/LiamBindle/PyVESC.git@master
    to install the pyvesc library
    '''
    def __init__(self, serial_port, percent=.15, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05, steering_scale = 1.0, steering_offset = 0.05 ):
        
        try:
            import pyvesc
        except Exception as err:
            print("\n\n\n\n", err, "\n")
            print("please use the following command to import pyvesc so that you can also set")
            print("the servo position:")
            print("pip install git+https://github.com/LiamBindle/PyVESC.git@master")
            print("\n\n\n")
            time.sleep(1)
            raise
        
        assert percent <= 1 and percent >= -1,'\n\nOnly percentages are allowed for MAX_VESC_SPEED (we recommend a value of about .2) (negative values flip direction of motor)'
        self.steering_scale = steering_scale
        self.steering_offset = steering_offset
        self.percent = percent
        
        try:
            self.v = pyvesc.VESC(serial_port, has_sensor, start_heartbeat, baudrate, timeout)
        except Exception as err:
            print("\n\n\n\n", err)
            print("\n\nto fix permission denied errors, try running the following command:")
            print("sudo chmod a+rw {}".format(serial_port), "\n\n\n\n")
            time.sleep(1)
            raise
        
    def run(self, angle, throttle):
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)
        self.v.set_duty_cycle(throttle*self.percent)

    def left_turn(self, slope):
        angle = slope * 0.4
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)

    def right_turn(self, slope):
        angle = 0.5 + (1 - slope) * 0.4
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)


##Implementation

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)

# Create VESC object and start motor
Vesc_object = VESC('/dev/ttyACM0') # create a VESC object with the serial port and also specify the other keyword arguments if they are different from the 
#ones in your myconfig.py script

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # pid = PID(1, 0.1, 0.05, setpoint=1)
    print('Connected cameras: ', device.getConnectedCameras())
    # Print out usb speed
    print('Usb speed: ', device.getUsbSpeed().name)
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version: ', device.getBootloaderVersion())
    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    Vesc_object.run(0.5,0.2) #0.5 = straight, 0.1 is 10% throttle
    while True:
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
        image = inRgb.getCvFrame()
        ###### COLOR SEGMENTATION ######
        lane_image_2 = np.copy(image)
        # lane_image_2 = cv2.cvtColor(lane_image_2,cv2.COLOR_BGR2RGB)
        lane_image_2 =cv2.cvtColor(lane_image_2,cv2.COLOR_BGR2HSV)
        # show_image('input',lane_image_2)
        # show_image('hsv_input',lane_image_2)
        print("DISPLAY")
        plt.imshow(lane_image_2)
        plt.show()
        #Defining upper and lower boundaries for white color
        lower_white_hls = np.uint8([  0, 170,   0])
        upper_white_hls = np.uint8([5, 250, 255])
        # #Using bitwise operators to segment out white colors
        lane_white_mask = cv2.inRange(lane_image_2,lower_white_hls,upper_white_hls)
        show_image('whitemask',lane_white_mask)
        kernel = np.ones((15,15),np.uint8)
        lane_image_2 = cv2.morphologyEx(lane_white_mask, cv2.MORPH_CLOSE, kernel)
        kerenel_dilate = np.ones((7,7),np.uint8)
        lane_image_3 = cv2.dilate(lane_image_2,kerenel_dilate,iterations = 1)
        show_image('closing',lane_image_2)
        show_image('withdilation', lane_image_3)
        lane_image_mask = cv2.bitwise_and(lane_image_2,lane_image_2,mask=lane_white_mask)
        show_image('bitmask',lane_image_mask)

        ## back to line detection ##
        lane_image = np.copy(image)
        show_image('image',lane_image)
        lane_canny = find_canny(lane_image,100,200)
        # show_image('canny',lane_canny)
        lane_roi = region_of_interest(lane_canny)
        show_image('roi',lane_roi)
        lane_lines = cv2.HoughLinesP(lane_roi,1,np.pi/180,50,40,5)
        if lane_lines is not None:
            lane_lines_plotted = draw_lines(lane_image,lane_lines)
        else:
            continue
        show_image('lines',lane_lines_plotted)
        result_lines = compute_average_lines(lane_image,lane_lines)
        # print("Result", result_lines)
        final_lines_mask = draw_lines(lane_image,result_lines)
        # show_image('final',final_lines_mask)
        
        total_slope = []

        if len(result_lines) > 0:
            for points in result_lines:
                x1,y1,x2,y2 = points[0]
                if abs(x1-x2) > 0:
                    slope = (y1-y2)/(x1-x2)
                    total_slope.append(slope)

                cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
        total_slope = np.mean(total_slope)
        # total_slope = pid(total_slope)
        print("Total Slope: ", total_slope)
        if total_slope > 0:
            print("left")
            Vesc_object.left_turn(abs(total_slope))
        elif total_slope <= 0:
            print("right")
            Vesc_object.right_turn(abs(total_slope))
        # show_image('output',image)
        if cv2.waitKey(1) == ord('q'):
            break
    Vesc_object.run(0.5,0)
'''
# Video Processing:
cap = cv2.VideoCapture('output_Trim.mp4')
if not cap.isOpened:
    print('Error opening video capture')
    exit(0)
while True:
    ret, frame = cap.read()
    if frame is None:
        print(' No captured frame -- Break!')
        break
    lane_image = np.copy(frame)
    lane_canny = find_canny(lane_image,50,100)
    # show_image('canny',lane_canny)
    lane_roi = region_of_interest(lane_canny)
    # show_image('roi',lane_roi)
    lane_lines = cv2.HoughLinesP(lane_roi,1,np.pi/180,50,40,5)
    lane_lines_plotted = draw_lines(lane_image,lane_lines)
    # show_image('lines',lane_lines_plotted)
    result_lines = []
    final_lines_mask = []
    
    result_lines = compute_average_lines(lane_image,lane_lines)
    print("Result", result_lines)
    final_lines_mask = draw_lines(lane_image,result_lines)
    # show_image('final',final_lines_mask)
    
    total_slope = []

    for points in result_lines:
        x1,y1,x2,y2 = points[0]
        if abs(x1-x2) > 0:
            slope = (y1-y2)/(x1-x2)
            total_slope.append(slope)

        cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
    total_slope = np.mean(total_slope)
    if total_slope > 0:
        print("left")
    elif total_slope < 0:
        print("right")

    show_image('output',frame)
    if cv2.waitKey(1) == ord('q'):
        break
'''
