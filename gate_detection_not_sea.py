import sys

from interface.srv import Gate
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
import numpy as np
import math
import operator
from PIL import Image
# from cv_bridge import CvBridge , CvBridgeError



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('gate_detection')
        self.cli = self.create_client(Gate, 'gate_location')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Gate.Request()
        self.declare_parameter("img_src")

    def send_request(self):
    	
        img_src_param = self.get_parameter("img_src").get_parameter_value().string_value
        orig_frame =  cv2.imread(img_src_param)
        
        detector = GateDetector()

    
        #cv2.imshow("original", frame )
        #cv2.waitKey(0)
        #cv2.destroyAllWindows() 

        self.req.position =  detector.process(orig_frame) # returns int value (box number)
        self.future = self.cli.call_async(self.req)



###########################################################################
def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

def hlsSplitting(image):
    converted = convert_hls(image)
    h,l,s = cv2.split(converted)
    return s,l

def hsvSplitting(image):
    converted = convert_hls(image)
    h,s,v = cv2.split(converted)
    return s,v

def select_black_yellow(image):
    
    masked = cv2.inRange(image,20,100)
    
    return masked

def edges (l_rMasked,x,y) :
    sobel = cv2.Sobel(l_rMasked , cv2.CV_64F, x,y)
    abs_sobel = np.absolute(sobel)
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    return scaled_sobel

def angle(rect):
    """
    Produce a more useful angle from a rotated rect. This format only exists to make more sense to the programmer or
    a user watching the output. The algorithm for transforming the angle is to add 180 if the width < height or
    otherwise add 90 to the raw OpenCV angle.
    :param rect: rectangle to get angle from
    :return: the formatted angle
    """
    if rect[1][0] < rect[1][1]:
        return rect[2] + 180
    else:
        return rect[2] + 90

def distance(a, b):
    """
    Calculate the distance between points a & b
    :return: distance as given by the distance formula: sqrt[(a.x - b.x)^2 + (a.y - b.y)^2]
    """
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))

def hullx_score(hull):

  if (cv2.contourArea(hull)< 500):
    return 0

  score = 0

  rect = cv2.minAreaRect(hull) 
  short = min(rect[1])
  longe = max(rect[1])
  theta = angle(rect)

  if (longe >= 13.3 * short) and (longe <= 18.8 * short) :
    score = score + cv2.contourArea(hull)  
    if ( theta in range (-8,8)) or ( theta in range (172,188)) :
      score = score + cv2.contourArea(hull)   

  else:
    return 0     # score based on angle

  return score


def hully_score(hull,left,right):
  if (cv2.contourArea(hull)< 1500  ):
    return 0

  score =0  
  rect = cv2.minAreaRect(hull) 
  short = min(rect[1])
  longe = max(rect[1])
  cx = int (rect[0][0])
  cy = int (rect[0][1])
  theta = angle(rect)

  if (longe < 15 * short) or (longe > 24 *short):
    return 0

  score = cv2.contourArea(hull)
  
  if (left != None ):
    Hleft = max(left[1])
    if ( abs(angle(left)- theta) in range(89,91)):
      score += cv2.contourArea(hull) *2
    if (cx > left[0][0]):
      score += cv2.contourArea(hull) 
    if (cy  in range(int(left[0][1]-(Hleft//2) - 80) ,int(left[0][1] -(Hleft//2))) ):
      score += cv2.contourArea(hull)*4


  if (right != None ):
    Hright = max(right[1])
    if ( abs(angle(right)- theta) in range(89,91)):
      score += cv2.contourArea(hull) *2
    if (cx < right[0][0]):
      score += cv2.contourArea(hull)  
    if (cy  in range(int(right[0][1]-(Hright//2) - 80 ) ,int(right[0][1]-(Hright//2)) ) ):
      score += cv2.contourArea(hull) *4

  if (right != None  and left != None ):
    if (cx in range(int(left[0][0]),int(right[0][0])) ): 
      score += cv2.contourArea(hull) *4 

       


  return score

def hulls_score(hull,left,right,horizontal):

  if (cv2.contourArea(hull)< 400 ):
    return 0 

  score = 0

  rect = cv2.minAreaRect(hull)
  short = min(rect[1])
  longe = max(rect[1])
  cx = int (rect[0][0])
  cy = int (rect[0][1])
  theta = angle(rect)
  score = cv2.contourArea(hull)
  
  if (longe >=  4* short) and (longe <= 26*short):
    score += cv2.contourArea(hull)
    if ( theta in range(-6,6) or theta in range(-174,184)):
      score += cv2.contourArea(hull) *3

    if (left != None):
      if ( abs(angle(left)- theta ) in range(0,2) or abs(angle(left)- theta ) in range(178,182) ):
        score += cv2.contourArea(hull)*2
      if (cx > left[0][0]):
        score +=   cv2.contourArea(hull)

    if (right != None):
      if (cx < right[0][0]):
        score +=   cv2.contourArea(hull)


    if ( horizontal != None ):
      if ( abs(angle(horizontal)- theta ) in range(88,92)):
        score += cv2.contourArea(hull)
      if (cy > horizontal[0][1] ):
        score +=   cv2.contourArea(hull) 

    if ( horizontal != None  and left != None):
      if (cy > horizontal[0][1] and cy < left[0][1]):
        score +=   cv2.contourArea(hull) * 3

    if ( right != None  and left != None):
      if (cx in range(int(left[0][0]),int(right[0][0])) ):
        score +=   cv2.contourArea(hull) * 3          


  else : 
    return 0


  
  return score  



def contouring (imagex,imagey):

    print(type(imagex))
    print(type(imagey))
    contoursx, hierarchyx = cv2.findContours(imagex, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoursy, hierarchyx = cv2.findContours(imagey, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contoursx, contoursy

def convex_hulls(contours):

    """
    Convenience method to get a list of convex hulls from list of contours
    :param contours: contours that should be turned into convex hulls
    :return: a list of convex hulls that match each contour
    """

    hulls = []
    for contour in contours:
        hulls.append(cv2.convexHull(contour))

    return hulls


def gateParts(hullsx,hullsy):

  scoresx = []
  for i in hullsx:
    scoresx.append(hullx_score(i))

  if (max(scoresx) != 0) :
    lidx  = np.argmax(scoresx)
    left = cv2.minAreaRect(hullsx[lidx])
    scoresx[lidx] = 0
    if (max(scoresx) != 0) :
      ridx  = np.argmax(scoresx)
      right = cv2.minAreaRect(hullsx[ridx])
      hullsx.pop(lidx)
      hullsx.pop(ridx)
    else :
      hullsx.pop(lidx)
      right = None  
  
  else :
    left = None
    right = None  

  ###########################################
  if (right != None ) and (left != None) :
   if right[0][0] < left[0][0]:
      left, right = right, left
########################################
  scoresy = []
  for i in hullsy:
    scoresy.append(hully_score(i,left,right))

  if (max(scoresy) !=0 ):
    horizontal = cv2.minAreaRect(hullsy[np.argmax(scoresy)])
  else :
    horizontal = None   

  ######################################
  scoress = []
  for j in hullsx:
    scoress.append(hulls_score(j,left,right,horizontal))

  if ( max(scoress) != 0):
    small = cv2.minAreaRect(hullsx[np.argmax(scoress)])
  else :
    small = None  


  return (right,left,small,horizontal)

def boundingBox(img,right,left,small,horizontal):
  
  if (right != None ) and (left != None) :
    cxl = left[0][0]
    cyl = left[0][1]
    hl  = max(left[1])
    tl =  min(left[1])

    cxr = right[0][0]
    cyr = right[0][1]

    start = ( int(cxl-tl/2) , int(cyl-hl/2-tl*1.2) )
    end  = (int(cxr+tl/2) , int(cyr+hl/2) )
    #print(start)
    #print(end)
    cv2.rectangle(img,start,end,(0,255,0),3)

  if (small != None ) and (left != None) :
    cxs = small[0][0]
    cys = small[0][1]
    hs  = max(small[1])
    ts =  min(small[1])

    cxl = left[0][0]
    start = (int(cxl-ts/2) ,int(cys-hs/2-ts*1.2) )
    end   = (int(cxs+ts/2) , int(cys+hs/2) )
    cv2.rectangle(img,start,end,(255,0,0),3)

  return  img





def isInside(start,end,x,y):

    return (x >= start[0] and x <= end[0] and y >= start[1] and y <= end[1])

#################################
class GateDetector(object):
    

   def process(self, image):
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        #print(type(image))
        s,l=hlsSplitting(image)
        sMask=select_black_yellow(s)
        lMask=select_black_yellow(l)
        r,c = lMask.shape[0],lMask.shape[1]
        lMask[ r//2 : -1 , : ] = 255
        verticalEdges=edges(sMask,1,0)
        #print(verticalEdges.dtype)
        horizentalEdges=edges(lMask,0,1)
        contoursx,_ = cv2.findContours(verticalEdges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursy,_ = cv2.findContours(horizentalEdges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hullsx = convex_hulls( contoursx)
        hullsy = convex_hulls(contoursy)

        right,left,small,horizontal=gateParts(hullsx,hullsy)
       
        img = boundingBox(image,right,left,small,horizontal)
        r = int(img.shape[1]//3)
        c = int(img.shape[0]//3)
        count = 1
        for i in range(3):
            for j in range(3):
                cv2.rectangle(img,(i*r,j*c),(i*r+r,j*c+c),(255,120,0),3)

        Image.fromarray(img).show() # show image in run time
        for i in range(3):
            for j in range(3): 
                if(isInside((i*r,j*c)  , (i*r+r,j*c+c) , left[0][0] , left[0][1] ) ):
                    return count
                count = count + 1

   

###############################################################################################



def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Movements: %s' %
                    (response.movement))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


