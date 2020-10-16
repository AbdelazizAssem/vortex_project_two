import sys

from interface.srv import Gate
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
import numpy as np
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

        x_rec = 0
        w_rec = 0
        y_rec = 0
        h_rec = 0
        
        orig_frame =  cv2.imread(img_src_param)
        orig_frame = cv2.resize(orig_frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        frame = cv2.GaussianBlur(orig_frame, (3,3), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h , s , v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        clahe_img = clahe.apply(v)
        
        edges = cv2.Canny(clahe_img, 75, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
        sh = lines.shape[0]
        s = np.zeros((sh,1) , dtype = 'float32')
        for i in range(sh):    
        	s[i] =  lines[i].sum(axis = 1)
	
	
        
        left = lines[np.argmin(s)]
        right = lines[np.argmax(s)]
        
        w_final = right[0][0] - left[0][0]
        h_rec = right[0][3] - right[0][1]
        cv2.rectangle(frame, (left[0][0], right[0][1]), (left[0][0] + w_final,right[0][1] + h_rec), (0, 0, 255), 2)
        
        if (lines is not None) :
        	for line in lines:
        		x1, y1, x2, y2 = line[0]
        
        		if (x1 < x2):
            			diff = x2-x1
        		else:
            			diff = x1-x2
        		if y1 < y2:
            			diff_y = y2-y1
                  
        		else:
            			diff_y = y1-y2
            			
        		if (diff < 5) and (diff_y > 10) and (x1 < right[0][0]) and (x1 > left[0][0] ):
        			w1 = x1 - left[0][0]
        			w2 = right[0][0] - x1
        			if w1 < w2 :
        				cv2.rectangle(frame, (left[0][0], right[0][1]), (left[0][0] + w1,right[0][1] + h_rec), (0,255,0), 2)
        				cv2.putText(frame, 'NARROW GATE ', (x1, right[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        			if w1 > w2:
        				top = right[0][0]
        				bottom = right[0][1]
        				cv2.rectangle(frame, (x1, right[0][1]), (x1 + w2 ,right[0][1] + h_rec), (0, 255, 0), 2)
        				cv2.putText(frame, 'NARROW GATE ', (x1, right[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                      
        
        
        if top <286 and top >0:
        	col =0
        if top <572 and top >286:
        	col = 1
        if top <860 and top >572:
        	col =2
        	
        if bottom <180 and bottom >0:
        	if 180 - bottom < 100:
	        	row =1
	        else:
	        	row = 0
        if bottom <360 and bottom >180:
        	if 360 - bottom < 100:
	        	row = 2
	        else:
	        	row = 1
        if bottom <540 and bottom >360:
        	row =2
        	
        	
        if row == 0 and col== 0:
        	roi=1
        if row == 0 and col==1:
        	roi = 2
        if row == 0 and col==2:
        	roi =3
        	
        if row == 1 and col==0:
        	roi =4
        if row == 1 and col==1:
        	roi = 5
        if row == 1 and col==2:
        	roi =6
        if row == 2 and col==0:
        	roi =7
        if row == 2 and col==1:
        	roi = 8
        if row == 2 and col==2:
        	roi =9	       

	
        #self.req.b = int(sys.argv[2])
        
	







        cv2.line(frame,(0,180),(860,180),(255,0,0),2)
        cv2.line(frame,(0,360),(860,360),(255,0,0),2)
        cv2.line(frame,(286,0),(286,540),(255,0,0),2)
        cv2.line(frame,(572,0),(572,540),(255,0,0),2)
        cv2.imshow("original", frame )
        cv2.waitKey(0)
        cv2.destroyAllWindows() 

        self.req.position = int(roi)
        self.future = self.cli.call_async(self.req)


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

