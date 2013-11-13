#!/usr/bin/env python
import roslib; roslib.load_manifest('controls_demo')
import rospy
import cv_bridge
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import Float32
import sys


#################################
###
### Author: Antonella Wilby
### Email: awilby@ucsd.edu
###
### EXPLAIN WHAT THIS DOES
###
#################################

### TO DO
# size windows based on screen size
# make windows/images smaller to fit on screen
# write good docstrings
# add controls window with sliders and preset thresholds buttons (red, green, blue, etc)
# get right image topic working


class TargetTracker:
    def __init__(self):
        """Initializes TargetTracker."""
        
        # Preset thresholds for targets of different colors in BGR
        # these are dummy numbers for now
        self.red_thresholds = [ cv.Scalar(0,20,70), cv.Scalar(80,90,90) ]
        self.green_thresholds = [ cv.Scalar(0,70,90), cv.Scalar(90,120,190) ]
        self.blue_thresholds = [ cv.Scalar(0,0,90), cv.Scalar(80,80,100) ]
        
        # Current thresholds: dynamically adjusted
        self.thresholds = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255, \
                       'low_hue':0, 'high_hue':255, \
                       'low_sat':0, 'high_sat':255, \
                       'low_val':0, 'high_val':255}
        
        # Initialize all GUI elements
        self.initialize_gui()
        
        # Bridge from incoming ROS images to OpenCV images
        self.bridge = cv_bridge.CvBridge()
    
        # Subscribe to rectified and debayered image topic from left camera NOTE: this is currently raw
        rospy.Subscriber('/stereo/left/image_raw', sm.Image, self.handle_left_camera)
    
        # Subscribe to rectified and debayered image topic from right camera NOTE: this is currently raw
        rospy.Subscriber('/stereo/right/image_rect_color', sm.Image, self.handle_right_camera)
        
        # Publish distance from target
        self.distance_pub = rospy.Publisher('distance_from_target', Float32)
        
        
    ### GUI INITIALIZATION ###
    def initialize_gui(self):
        """Initializes GUI:
                - Windows for incoming images from left camera and right camera
                - Windows for thresholded image from left camera and right camera
                - Control window containing threshold sliders and threshold auto-select buttons.
        """
        
        # Instantiate OpenCV windows for displaying incoming images
        cv.NamedWindow('Left Camera', 1)
        cv.MoveWindow('Left Camera', 0, 0)
        cv.NamedWindow('Right Camera', 2)
        cv.MoveWindow('Right Camera', 760, 0)
        
        # Set callback for mouse input to Left Camera window
        # Used to select regions of interest for target thresholding
        #cv.SetMouseCallback('Left Camera', self.handle_mouse_left_camera, None)
        
        # Instantiate OpenCV windows for displaying thresholded image
        cv.NamedWindow('Left Threshold', 3)
        cv.MoveWindow('Left Threshold', 0, 480)
        cv.NamedWindow('Right Threshold', 4)
        cv.MoveWindow('Right Threshold', 760, 480)
        
        # Instantiate controls window
        cv.NamedWindow('Threshold Controls', 5)
        cv.MoveWindow('Threshold Controls', 760, 540)
        
        # Create sliders for tuning RGB thresholds
        cv.CreateTrackbar('low_red', 'Threshold Controls', self.thresholds['low_red'], 255, lambda x: self.change_slider('low_red', x))
        cv.CreateTrackbar('high_red', 'Threshold Controls', self.thresholds['high_red'], 255, lambda x: self.change_slider('high_red', x))
        cv.CreateTrackbar('low_green', 'Threshold Controls', self.thresholds['low_green'], 255, lambda x: self.change_slider('low_green', x))
        cv.CreateTrackbar('high_green', 'Threshold Controls', self.thresholds['high_green'], 255, lambda x: self.change_slider('high_green', x))
        cv.CreateTrackbar('low_blue', 'Threshold Controls', self.thresholds['low_blue'], 255, lambda x: self.change_slider('low_blue', x))
        cv.CreateTrackbar('high_blue', 'Threshold Controls', self.thresholds['high_blue'], 255, lambda x: self.change_slider('high_blue', x))
        
        # Create buttons for auto-selecting red, green 
        #cv.CreateButton('red', 'Threshold Controls',  red_select, NULL, cv.CV_RADIOBOX)
        
    def change_slider(self, name, new_thresh):
        """Changes the slider values for a specified slider and the new threshold."""
        self.thresholds[name] = new_thresh
    
    
    ### CALLBACKS AND HANDLERS ###
    
    def handle_left_camera(self, data):
        """Handles incoming images from left stereo camera."""
        try:
            left_image = self.bridge.imgmsg_to_cv(data, 'bgr8')
        except cv_bridge.CvBridgeError, e:
            print e
        
        # Threshold image    
        threshed_image = self.threshold_image(left_image)
        
        # Calculate biggest region and display contours
        biggest_region = self.find_biggest_region(threshed_image)
        
        # Draw contours and bounding box on image
        self.draw_contours(left_image, biggest_region)

        # Show incoming image in Left Camera window
        cv.ShowImage('Left Camera', left_image)
        
        # Show thresholded image in Threshold window
        cv.ShowImage('Left Threshold', threshed_image)
        cv.WaitKey(3)


    def handle_right_camera(self, data):
        """Handles incoming images from right stereo camera."""
        try:
            right_image = self.bridge.imgmsg_to_cv(data, 'bgr8')
        except cv_bridge.CvBridgeError, e:
            print e
            
        cv.ShowImage('Right Camera', right_image)
        cv.WaitKey(3)


    def handle_mouse_left_camera(self, event, x, y, flags, param):
        """Handles incoming mouse input to the Left Camera window.
            Mouse input is used to select regions of interest (such as a colored target)
            and dynamically threshold image based on the colors in that region of interest."""
        
        down_coord = (0,0)
        up_coord = (0,0)
            
        # If the user depresses the left mouse button
        if event == cv.CV_EVENT_LBUTTONDOWN:
            down_coord = (x, y)
            
        elif event == cv.CV_EVENT_LBUTTONDOWN:
             up_coord = (x, y)
            
        # Put coordinates in order from lower x to higher x
        if down_coord[0] > up_coord[0]:
            down_coord[0], up_coord[0] = up_coord[0], down_coord[0]
        
        # Put coordinates in order from lower y to higher y
        if down_coord[1] > up_coord[1]:
            down_coord[1], up_coord[1] = up_coord[1], down_coord[1]
    
        selected_section = [(down_coord[0], up_coord[0]), (down_coord[1], up_coord[1])]
        
        self.process_selected_section(selected_section)
        
    
    ### IMAGE PROCESSING FUNCTIONS ###
    
    
    def process_selected_section(self, section):
        """ Thresholds image based on colors found on user-selected image section."""
        
        new_thresholds= { 'high_green':0, 'low_green':255,\
                         'low_blue':255, 'low_val':255,\
                         'high_hue':0, 'high_val':0,\
                         'low_hue':255, 'high_blue':0,\
                         'low_red':255,'high_red':0,\
                         'high_sat':0, 'low_sat':255}
        
        # Get coordinates of section in image
        x0, x1 = section[0][0], section[0][1]     # Begin and end x coordinates
        y0, y1 = section[1][0], section[1][1]     # Begin and end y coordinates
            
        if len(section) > 0:
            
            for x in range(x0, x1):
                for y in range(y0, y1):
                    (b, g, r) = image[y,x]      # Get RGB values at image coordinate
                
                    # Assign values to color names
                    color = { "blue": b, "green": g, "red": r}
                         
                    # Update thresholds for each color based on RGB values   
                    for name in color:
    
                        # String to get color key in dictionary
                        high_name = "high_" + name
                        low_name = "low_" + name
                              
                        # If RGB value at this pixel is greater than the max threshold value,
                        # update the max threshold value to this value
                        if color[name] > new_thresholds[high_name]:
                            new_thresholds[high_name] = color[name]
                             
                        # If RGB value at this pixel is less than the threshold value,
                        # update the min threshold value to this value
                        if color[name] < new_thresholds[low_name]:
                            new_thresholds[low_name] = color[name]
                                    

        # Now reset sliders to the found max and min
        self.thresholds = new_thresholds
    
    
    
    
    def threshold_image(self, image):
        """my name is ms. docstring i am married to mr. docstring"""
        
        # Create image to store thresholded image
        threshed_image = cv.CreateImage(cv.GetSize(image), 8, 1)
        
        # Get thresholds in BGR
        lower_thresh = cv.Scalar(self.thresholds['low_blue'],
                                 self.thresholds['low_green'],
                                 self.thresholds['low_red'])
        upper_thresh = cv.Scalar(self.thresholds['high_blue'],
                                 self.thresholds['high_green'],
                                 self.thresholds['high_red'])
        
        # Threshold image based on given ranges in BGR
        cv.InRangeS(image, lower_thresh, upper_thresh, threshed_image)
        
        return threshed_image
      
  
    def find_biggest_region(self, threshed_image):
        """Finds the biggest contour of all contours in thresholded image,
            then marks contour in the main image."""
        
        # Create storage in memory for all contours found
        memStorage = cv.CreateMemStorage(0)
        
        # Create copy of threshed_image in which to draw contours
        thresh_copy = cv.CreateImage(cv.GetSize(threshed_image), 8, 1)
        cv.Copy(threshed_image, thresh_copy)
    
        # Find all contours using OpenCV's built-in function
        contours = cv.FindContours(thresh_copy, memStorage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour
        if len(contours) > 0:
            biggest_region = contours
            biggest_area = cv.ContourArea(contours)
            while contours != None:
                next_area = cv.ContourArea(contours)
                if biggest_area < next_area:
                    biggest_region = contours
                    biggest_area = next_area
                contours = contours.h_next()
                
        else:
            biggest_region = 0
                
        # Return biggest region
        return biggest_region
    
    def draw_contours(self, image, biggest_region):
        """Draws contours of biggest region on image, then draws bounding box for the
            contour on image."""
            
        # Draw contours of biggest region on image
        cv.DrawContours(image, biggest_region, cv.RGB(255,255,255), \
                        cv.RGB(0,255,0), 1, thickness=2, lineType=8, offset=(0,0))
        
        # Draw bounding box in yellow of biggest region on image
        bound_box = cv.BoundingRect(biggest_region, update=0)
        cv.PolyLine( image, [[ (bound_box[0], bound_box[1]), \
                    (bound_box[0]+bound_box[2], bound_box[1]), \
                    (bound_box[0]+bound_box[2], bound_box[1]+bound_box[3]),\
                    (bound_box[0], bound_box[1]+bound_box[3]) ]], 1, cv.RGB(255,255,0) )
    
    
    def distance_from_blob_size(self, blob_size):
        """hello i am a docstring"""
        pass
    
    


    # Function to enable passing instance of DemoController
    def __repr__(self):
        return str(vars(self))



def main(args):
    """Initializes controls demo node and DemoController object."""
    
    # Initialize Controls Demo node
    rospy.init_node('target_tracker')
    
    # Initialize TargetTracker
    tracker = TargetTracker()
  
    # Run until program is quit
    rospy.spin()
    
    # Destroy windows after quitting 
    cv.DestroyAllWindows()


# Passes main system arguments if run standalone
if __name__ == "__main__":
    main(sys.argv)
