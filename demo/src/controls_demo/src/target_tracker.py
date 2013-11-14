#!/usr/bin/env python
import roslib; roslib.load_manifest('controls_demo')
import rospy
import cv_bridge
import cv
import cv2
import numpy
import sensor_msgs.msg as sm
from std_msgs.msg import Float32
import sys


###
### Author: Antonella Wilby
### Email: awilby@ucsd.edu
###
### EXPLAIN WHAT THIS DOES
###
### All images processed in HSV for better color segmentation performance.
###

### TO DO
# size windows based on screen size
# make windows/images smaller to fit on screen
# write good docstrings
# add controls window with preset thresholds buttons for target colors (red, green, blue, etc)
# get right image topic working
# ***NEXT: Implement blob detection
# change OpenCV sliders to dynamic_reconfigure_gui because they suck


class TargetTracker:
    def __init__(self):
        """Initializes TargetTracker."""
        
        # Preset thresholds for red, green, or blue targets
        # these are dummy numbers for now
        self.red_threshold = [ numpy.array([0,20,70]), numpy.array([80,90,90]) ]
        self.green_threshold = [ numpy.array([0,70,90]), numpy.array([90,120,190]) ]
        self.blue_threshold = [ numpy.array([110,50,50]), numpy.array([130,255,255]) ]

        # Current thresholds: dynamically adjusted
        self.low_thresholds = {'hue': 0, 'val': 0, 'sat': 0}
        self.high_thresholds = {'hue': 179, 'val': 255, 'sat': 255}
        
        # Variables for user-selected threshold selection          
        self.down_coord = (-1,-1)         # Negative since area hasn't been selected yet
        self.up_coord = (-1,-1)           # Negative since area hasn't been selected yet
        
        # Initialize all GUI elements
        self.initialize_gui()
        
        # Bridge from incoming ROS images to OpenCV images
        self.bridge = cv_bridge.CvBridge()
    
        # Subscribe to rectified and debayehue image topic from left camera NOTE: this is currently raw
        rospy.Subscriber('/stereo/left/image_raw', sm.Image, self.handle_left_camera)
    
        # Subscribe to rectified and debayehue image topic from right camera NOTE: this is currently raw
        #rospy.Subscriber('/stereo/right/image_raw', sm.Image, self.handle_right_camera)
        
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
        cv.SetMouseCallback('Left Camera', self.handle_mouse_left_camera, None)
        
        # Instantiate OpenCV windows for displaying thresholded image
        cv.NamedWindow('Left Threshold', 3)
        cv.MoveWindow('Left Threshold', 0, 480)
        cv.NamedWindow('Right Threshold', 4)
        cv.MoveWindow('Right Threshold', 760, 480)
        
        # Instantiate controls window
        cv.NamedWindow('Threshold Controls', 5)
        cv.MoveWindow('Threshold Controls', 760, 540)
        
        # Create sliders for tuning RGB thresholds
        cv.CreateTrackbar('low_hue', 'Threshold Controls', self.low_thresholds['hue'], 179, lambda x: self.change_slider('low', 'hue', x))
        cv.CreateTrackbar('high_hue', 'Threshold Controls', self.high_thresholds['hue'], 179, lambda x: self.change_slider('high', 'hue', x))
        cv.CreateTrackbar('low_sat', 'Threshold Controls', self.low_thresholds['sat'], 255, lambda x: self.change_slider('low', 'sat', x))
        cv.CreateTrackbar('high_sat', 'Threshold Controls', self.high_thresholds['sat'], 255, lambda x: self.change_slider('high','sat', x))
        cv.CreateTrackbar('low_val', 'Threshold Controls', self.low_thresholds['val'], 255, lambda x: self.change_slider('low','val', x))
        cv.CreateTrackbar('high_val', 'Threshold Controls', self.high_thresholds['val'], 255, lambda x: self.change_slider('high','val', x))
        
        # Create buttons for auto-selecting hue, sat 
        #cv.CreateButton('hue', 'Threshold Controls',  hue_select, NULL, cv.CV_RADIOBOX)
        
        
    def change_slider(self, hiLo, val, new_thresh):
        """Changes the slider values for a specified slider and the new threshold."""
        
        if hiLo == 'low':
            self.low_thresholds[val] = new_thresh
        elif hiLo == 'high':
            self.high_thresholds[val] = new_thresh
    
    
    ### CALLBACKS AND HANDLERS ###
    
    def handle_left_camera(self, data):
        """Handles incoming images from left stereo camera."""
        try:
            left_image = self.bridge.imgmsg_to_cv(data, 'bgr8')
        except cv_bridge.CvBridgeError, e:
            print e
            
        # Convert incoming image (CvMat) to numpy array
        left_image = numpy.asarray(left_image)
        
        # Convert image to HSV for better color segmentation
        self.left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2HSV)
        
        # Threshold image    
        threshed_image = self.threshold_image(self.left_image)
        
        # Calculate biggest contour and display contours
        biggest_region = self.find_biggest_contour(threshed_image, self.left_image)
        
        # Show incoming image in Left Camera window
        cv2.imshow('Left Camera', self.left_image)
        
        # Show thresholded image in Threshold window
        cv2.imshow('Left Threshold', threshed_image)
        cv.WaitKey(3)


    def handle_right_camera(self, data):
        """Handles incoming images from right stereo camera."""
        try:
            self.right_image = self.bridge.imgmsg_to_cv(data, 'bgr8')
        except cv_bridge.CvBridgeError, e:
            print e
            
        cv2.imshow('Right Camera', self.right_image)
        cv.WaitKey(3)


    def handle_mouse_left_camera(self, event, x, y, flags, param):
        """Handles incoming mouse input to the Left Camera window.
            Mouse input is used to select regions of interest (such as a colohue target)
            and dynamically threshold image based on the colors in that region of interest."""
            
        # If the user depresses the left mouse button
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.down_coord = (x, y)
        
        # If the user releases the left mouse button
        elif event == cv.CV_EVENT_LBUTTONUP:
            self.up_coord = (x, y)
            
            # Put coordinates in order from lower x to higher x
            if self.down_coord[0] > self.up_coord[0]:
                self.down_coord[0], self.up_coord[0] = self.up_coord[0], self.down_coord[0]
            
            # Put coordinates in order from lower y to higher y
            if self.down_coord[1] > self.up_coord[1]:
                self.down_coord[1], self.up_coord[1] = self.up_coord[1], self.down_coord[1]
        
            selected_section = [(self.down_coord[0], self.up_coord[0]), (self.down_coord[1], self.up_coord[1])]
            
            self.process_selected_section(selected_section, self.left_image)
        
    
    
    ### IMAGE PROCESSING FUNCTIONS ###
     
    def process_selected_section(self, section, image):
        """ Thresholds image based on colors found on user-selected image section."""
        
        new_low_thresh = {'hue': 179, 'sat': 255, 'val': 255}
        new_high_thresh = {'hue': 0, 'sat': 0, 'val': 0}
        
        # Get coordinates of section in image
        x0, x1 = section[0][0], section[0][1]     # Begin and end x coordinates
        y0, y1 = section[1][0], section[1][1]     # Begin and end y coordinates
            
        if len(section) > 0:
            
            for x in range(x0, x1):
                for y in range(y0, y1):
                    (h,s,v) = image[y,x]      # Get HSV values at image coordinate
                
                    # Assign values to color names
                    color = { "hue": h, "sat": s, "val": v}
                         
                    # Update thresholds for each color based on HSV values   
                    for name in color:
     
                        # If RGB value at this pixel is greater than the max threshold value,
                        # update the max threshold value to this value
                        if color[name] > new_high_thresh[name]:
                            new_high_thresh[name] = color[name]
                             
                        # If RGB value at this pixel is less than the threshold value,
                        # update the min threshold value to this value
                        if color[name] < new_low_thresh[name]:
                            new_low_thresh[name] = color[name]
                                    

        # Now reset sliders to the found max and min
        for name in {"hue", "sat", "val"}:
            self.change_slider('low', name, new_low_thresh[name])
            self.change_slider('high', name, new_high_thresh[name])
        
    
    
    def threshold_image(self, image):
        """my name is ms. docstring i am married to mr. docstring"""
        
        # Create image to store thresholded image
        #threshed_image = cv.CreateImage(cv.GetSize(image), 8, 1)
        
        # Get thresholds for selected color
        lower_thresh = [ self.low_thresholds['hue'], self.low_thresholds['sat'], self.low_thresholds['val'] ]
        upper_thresh = [ self.high_thresholds['hue'], self.high_thresholds['sat'], self.high_thresholds['val'] ]
        lower_thresh = numpy.array( lower_thresh )
        upper_thresh = numpy.array( upper_thresh )
        
        # Threshold image based on given ranges in HSV
        threshed_image = cv2.inRange(image, lower_thresh, upper_thresh)
        
        return threshed_image

  
    def find_biggest_contour(self, threshed_image, image):
        """Finds the biggest contour of all contours in thresholded image,
            then marks contour in the main image.
            Next, draws contours of the biggest region on the original image, then draws
            bounding box for the biggest region on the original image."""
        
        # Create storage in memory for all contours found
        memStorage = cv.CreateMemStorage(0)
        
        # Find all contours using OpenCV's built-in function
        contours, _ = cv2.findContours(threshed_image.copy(), cv2.RETR_LIST, \
                                   cv2.CHAIN_APPROX_SIMPLE)

        biggest_contour_area = 0
        biggest_contour = None
        
        # Find biggest contour
        for i in range(0, len(contours)):
            area = cv2.contourArea(contours[i], False)
            if area > biggest_contour_area:
                biggest_contour_area = area
                biggest_contour = contours[i]
                biggest_contour_indx = i
        
            
        # Draw contours and bounding box of biggest contour on image
        if biggest_contour is not None:
            # Draw contours of biggest region on image
            cv2.drawContours(image, contours, biggest_contour_indx, (255,255,255), 2)
        
            # Draw bounding box in yellow of biggest contour on image
            #bound_box = cv2.boundingRect(biggest_contour)
            #cv2.rectangle(image, bound_box, (255,255,0), 1, 8, 0)
    
                
        # Return biggest region
        return biggest_contour
    
    
    def find_blob(self):
        pass
    
    
    def blob_tracker(self):
        pass    
    
    
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
