import cv2
import time
import numpy as np
import apriltag

def show_image(name,img): #function for displaying the image
    cv2.imshow(name,img)
    cv2.waitKey(50)
    #cv2.destroyAllWindows()

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

    def set_vel(self, throttle):
        self.v.set_duty_cycle(throttle*self.percent)
      
    # absolute value of slope goes from 0 to 0.04
    def left_turn(self, slope):
        if(abs(slope) > 0.05):
            slope = 0.05
        angle = 0.5 - abs(slope) * 10
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)

    def right_turn(self, slope):
        if(abs(slope) > 0.05):
            slope = 0.05
        angle = 0.5 + abs(slope) * 10
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)


class velCurve():
  curveType =   None
  dist      =   None
  velMin    =   None
  velMax    =   None
  distance  =   None

  velocity  =   None
  position  =   None

  tagID     =   None

  next      =   None

  def __init__(self, tagID, Velocity_curve_type, velocity_min, velocity_max, motion_distance):
    self.curveType = Velocity_curve_type
    self.velMin = velocity_min
    self.velMax = velocity_max
    self.distance = motion_distance    

    self.position = 0
    self.slope = 0
    self.tagID = tagID

  def getVel(self, time):
    """
    TODO
    It would be nice to add some more velocity curve types

    """
    # Check if velocity is within its bounds
    if self.position < self.distance:
      if self.curveType == 'constant':
        # assign velocity to max
        self.velocity = self.velMax
        self.position += time*self.velocity
        #print(self.position)
        return [self.velMax, 0]
      if self.curveType == 'symmetric-ramp':
        # velocity ramps between min and max peaking in the middle
        if self.position == 0:
          self.velocity = self.velMin
          rise = self.velMax - self.velMin
          self.slope = (2*rise)/self.distance
        else:
          pass
        # ramp up on the first half 
        if self.position <= self.distance/2:
          self.velocity += self.slope*time
          self.position += time*self.velocity
          #print(self.velocity)
          #print(self.position)
        # ramp down in the middle half
        elif self.position > self.distance/2:
          self.velocity -= self.slope*time
          self.position += time*self.velocity
          #print(self.velocity)
          #print(self.position)
        else:
          # Poor man's exception handeling 
          print("something unexpected happened in getVel/ramp method")
      if self.curveType == 'sine':
        print("This feature has not been implemeted yet :/")
        pass        
    #If velocity is not within bounds of this object, pass to the next one
    else:
      try:
        self.position = 0
        return [self.velMin,1] 
      except:
        print("ERROR: Failed to hand over velocity to next object \n in getVel method in velCurve Class")
        return [self.velMin, 1]
      #Need to accomodate for handing off between velocity curves


class tagReader():
  detector    =   None
  image       =   None
  results     =   None
  tagID       =   list()
                      # curve_type minVel maxVel linear Distance 
  nodes = {0 : velCurve(0, "constant", 0.2, 0.35, 1.683),
           1 : velCurve(1, "constant", 0.2, 0.2, 9.650),
           2 : velCurve(2, "constant", 0.2, 0.35, 2.280),
           3 : velCurve(3, "constant", 0.2, 0.2, 1.918),
           4 : velCurve(4, "constant", 0.2, 0.35, 1.280),}
  
  def __init__(self, detector):
    self.detector = detector
  def setImage(self, img):
    self.image = img
  def readImage(self):
    img = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
    #Getting the resutls from the detector, with the image input
    self.results = self.detector.detect(img)
    for r in self.results:
      self.tagID.append(int(r.tag_id))
    try:
      return self.nodes[self.tagID[len(self.tagID)-1]]
    except:
      print("ERROR: failed to detect node \n in read image method in tagReader class")
      return None
  def annotateImage(self):
    img = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) # Converting the origonal image back to color, so that we can draw bounding boxes and stuff

    if self.results == None:
        self.readImage()
    
    #loop to draw bounding boxes, and mark centers
    for r in self.results:
      # Setting corners
      (A,B,C,D) = r.corners 
      corner_B = (int(B[0]), int(B[1]))
      corner_C = (int(C[0]), int(C[1]))
      corner_D = (int(D[0]), int(D[1]))
      corner_A = (int(A[0]), int(A[1]))
      
      #print("a", corner_A, "red")
      #print("b", corner_B, "green")
      #print("c", corner_C, "blue")
      #print("d", corner_D, "purple")

      cv2.circle(img,corner_A, 5, (0, 0, 255), -1)
      cv2.circle(img,corner_B, 5, (0, 255, 0), -1)
      cv2.circle(img,corner_C, 5, (255, 0, 0), -1)
      cv2.circle(img,corner_D, 5, (255, 0, 255), -1)

      # Drawing lines between corners
      #cv2.line(img, corner_A, corner_B, (0,255,0), 5)
      #cv2.line(img, corner_B, corner_C, (0,255,0), 5)
      #cv2.line(img, corner_C, corner_D, (0,255,0), 5)
      #cv2.line(img, corner_D, corner_A, (0,255,0), 5)

      # Draw a circle in the center of each tag detected
      (cX,cY) = (int(r.center[0]), int(r.center[1]))
      cv2.circle(img,(cX,cY), 5, (0, 0, 255), -1)

      # Writing the detected tagIDs on the image
      tagFamily = r.tag_family.decode("utf-8")
      tagID = str(r.tag_id)
      print(tagID)
      cv2.putText(img, "tagID: " + tagID, (corner_A[0]+20, cY),
                  cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 5)
      
      print("[INFO] tag ID: {}".format(tagID))
      #return img
      return cv2.resize(img[int(A[1]):int(D[1]),int(A[0]):int(B[0])],(200,200), interpolation = cv2.INTER_AREA)