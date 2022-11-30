import cv2 as cv2 #importing the library
import numpy as np
import matplotlib.pyplot as plt
import depthai as dai
import math
import time

# TODO - calc slopes and make VESC steering react to slope values

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
    
    bounds = np.array([[[0,image.shape[0]],[0,image.shape[0]/2],[1920,image.shape[0]/2],[1920,image.shape[0]]]],dtype=np.int32)
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

    if len(left_lane_lines) < 1:
        print("left lane not detected")

        #Computing average slope and intercept
        #left_average_line = np.average(left_lane_lines,axis=0)
        left_average_line = [0,0]
        right_average_line = np.average(right_lane_lines,axis=0)
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


    if len(right_lane_lines) < 1:
        print("right lane not detected")

        #Computing average slope and intercept
        left_average_line = np.average(left_lane_lines,axis=0)
        #right_average_line = np.average(right_lane_lines,axis=0)
        right_average_line = [0,0]
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

    else:
        #Computing average slope and intercept
        left_average_line = np.average(left_lane_lines,axis=0)
        right_average_line = np.average(right_lane_lines,axis=0)
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
    def __init__(self, serial_port, percent=.2, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05, steering_scale = 1.0, steering_offset = 0.0 ):
        
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

'''
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

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

	print('Connected cameras: ', device.getConnectedCameras())
    # Print out usb speed
	print('Usb speed: ', device.getUsbSpeed().name)
    # Bootloader version
	if device.getBootloaderVersion() is not None:
		print('Bootloader version: ', device.getBootloaderVersion())

    # Output queue will be used to get the rgb frames from the output defined above
	qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

	while True:
		inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

		image = inRgb.getCvFrame()
		lane_image = np.copy(image)
		lane_canny = find_canny(lane_image,100,200)
		#cv2.imshow('Canny',lane_canny)
		lane_roi = region_of_interest(lane_image)
		cv2.imshow("ROI",lane_roi)
		# lane_lines = cv2.HoughLinesP(lane_roi,1,np.pi/180,50,40,5)
		# print(lane_lines)
		# lane_lines_plotted = draw_lines(lane_image,lane_lines)
		# cv2.imshow('lines',lane_lines_plotted)
		# result_lines = compute_average_lines(lane_image,lane_lines)
		# print(result_lines)
		# final_lines_mask = draw_lines(lane_image,result_lines)
		# show_image('final',final_lines_mask)

		# for points in result_lines:
		# 	x1,y1,x2,y2 = points[0]
		# 	cv2.line(lane_image,(x1,y1),(x2,y2),(0,0,255),5)

		# show_image('output',lane_image)
		if cv2.waitKey(1) == ord('q'):
			break
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
