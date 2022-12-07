import cv2
import depthai as dai
import apriltag
import time
import numpy as np
from autodriver_methods import VESC
from autodriver_methods import velCurve
from autodriver_methods import tagReader
from autodriver_methods import region_of_interest

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

# AprilTag initialization
apriltag.DetectorOptions() # Setting up the detector options object
options = apriltag.DetectorOptions(families="tag16h5",quad_decimate=1)
detector = apriltag.Detector(options) # Instantiating the detector, with the detector options
reader = tagReader(detector)
frequency = 1/20 # (seconds)

# Create and initialize VESC Object
Vesc_object = VESC('/dev/ttyACM0')

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

    Vesc_object.run(0.5,0.2) #0.5 = straight, 0.2 is 20% throttle

    while True:
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

        image = inRgb.getCvFrame()
        reader.setImage(image)

        ###### COLOR SEGMENTATION ######
        lane_image = np.copy(image)
        # lane_image = cv2.cvtColor(lane_image,cv2.COLOR_BGR2RGB)
        image_hsv = cv2.cvtColor(lane_image,cv2.COLOR_BGR2HSV)
        image_roi = region_of_interest(image_hsv)
        # show_image('input',lane_image)
        # show_image('hsv_input',lane_image)
        #print("DISPLAY")
        #plt.imshow(lane_image)
        #plt.show()

        #Defining upper and lower boundaries for white color
        lower_white_hls = np.uint8([  0, 170,   0])
        upper_white_hls = np.uint8([5, 250, 255])

        upper_green_mask = np.uint8([100,250,255])
        lower_green_mask = np.uint8([40,0,10])
        upper_blue_mask = np.uint8([135,250,255])
        lower_blue_mask = np.uint8([100,75,20])

        pm_upper_green_mask = np.uint8([80,250,255])
        pm_lower_green_mask = np.uint8([40,75,10])
        pm_upper_blue_mask = np.uint8([135,250,255])
        pm_lower_blue_mask = np.uint8([100,75,20])

        lane_blue_mask = cv2.inRange(image_roi, lower_blue_mask, upper_blue_mask)
        lane_green_mask = cv2.inRange(image_roi, pm_lower_green_mask, upper_green_mask)
        combined_mask = lane_blue_mask + lane_green_mask
        #show_image('image',image)
        #show_image('blue mask',lane_blue_mask)
        #show_image('green mask',lane_green_mask)
        #show_image('combined mask',combined_mask)

        ######### Turn Logic #########
        num_green_mask = np.sum(lane_green_mask == 255)
        num_blue_mask = np.sum(lane_blue_mask == 255)
        pixles = 1920*1080/3
        norm_blue = num_blue_mask/(pixles)
        norm_green = num_green_mask/(pixles)

        gb_diff = norm_green - norm_blue
            
        print("gb_diff: ",gb_diff)
        if gb_diff < 0:
            print("left")
            Vesc_object.left_turn(gb_diff)
        elif gb_diff >= 0:
            print("right")
            Vesc_object.right_turn(gb_diff)

        #### APRILTAG VELOCITY CONTROL ####
        velGen = reader.readImage()
        if velGen is not None:
            parameters = velGen.getVel(frequency)
            velocity, done = parameters
            print(velGen.tagID)
            print("Velocity: ", velocity)
            Vesc_object.set_vel(velocity)
            while done != 1:
                #print("carrying out action...")
                done = velGen.getVel(frequency)[1]

        if cv2.waitKey(1) == ord('q'):
            break
