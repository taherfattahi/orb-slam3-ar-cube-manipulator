#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from std_msgs.msg import Header

class PinchGestureDetector:

    def numpy_to_image_msg(self, img, frame_id="camera_frame"):
        """Convert a numpy array to a ROS image message without using cv_bridge."""
        if img.dtype != np.uint8:
            img = np.array(img, dtype=np.uint8)
    
        ros_img = Image()
        ros_img.header = Header()
        ros_img.header.stamp = rospy.Time.now()
        ros_img.header.frame_id = frame_id
        ros_img.height = img.shape[0]
        ros_img.width = img.shape[1]
    
        if len(img.shape) == 2:  # Grayscale
            ros_img.encoding = "mono8"
            ros_img.step = img.shape[1]
            ros_img.data = img.tobytes()
        elif len(img.shape) == 3:  # Color
            if img.shape[2] == 3:
                ros_img.encoding = "bgr8"
                ros_img.step = img.shape[1] * 3
                ros_img.data = img.tobytes()
            elif img.shape[2] == 4:
                ros_img.encoding = "bgra8"
                ros_img.step = img.shape[1] * 4
                ros_img.data = img.tobytes()
    
        return ros_img
    
    def __init__(self):
        rospy.init_node('pinch_gesture_detector', anonymous=True)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use 0 for default camera
        self.bridge = CvBridge()
        
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # ROS publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        
        # Translation pinch publishers (thumb-index)
        self.pinch_position_pub = rospy.Publisher('/pinch_position', Point, queue_size=10)
        self.pinch_active_pub = rospy.Publisher('/pinch_active', Bool, queue_size=10)
        
        # Rotation pinch publishers (thumb-middle)
        self.rotation_pinch_position_pub = rospy.Publisher('/rotation_pinch_position', Point, queue_size=10)
        self.rotation_pinch_active_pub = rospy.Publisher('/rotation_pinch_active', Bool, queue_size=10)
        
        # Pinch states
        self.pinch_active = False
        self.rotation_pinch_active = False
        self.prev_pinch_pos = None
        self.prev_rotation_pinch_pos = None
        
        # Debug window
        self.show_debug_window = True
        
        # Main loop
        self.process_camera()
    
    def detect_pinches(self, hand_landmarks, img_shape):
        h, w, _ = img_shape
        
        # Get finger positions
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        
        # Convert to pixel coordinates for visualization
        thumb_x, thumb_y = int(thumb_tip.x * w), int(thumb_tip.y * h)
        index_x, index_y = int(index_tip.x * w), int(index_tip.y * h)
        middle_x, middle_y = int(middle_tip.x * w), int(middle_tip.y * h)
        ring_x, ring_y = int(ring_tip.x * w), int(ring_tip.y * h)
        
        # Calculate distances
        thumb_index_distance = np.sqrt((thumb_tip.x - index_tip.x)**2 + 
                                     (thumb_tip.y - index_tip.y)**2)
        
        # Calculate distance for rotation pinch (thumb to middle finger)
        thumb_middle_distance = np.sqrt((thumb_tip.x - middle_tip.x)**2 + 
                                      (thumb_tip.y - middle_tip.y)**2)
        
        # Other finger distances for validation
        index_middle_distance = np.sqrt((index_tip.x - middle_tip.x)**2 + 
                                      (index_tip.y - middle_tip.y)**2)
        middle_ring_distance = np.sqrt((middle_tip.x - ring_tip.x)**2 + 
                                     (middle_tip.y - ring_tip.y)**2)
        
        # Pinch thresholds
        translation_pinch_threshold = 0.07
        rotation_pinch_threshold = 0.07
        
        # TRANSLATION PINCH (thumb-index)
        # For translation, require index-middle to be separated (to distinguish from rotation pinch)
        valid_translation_separation = index_middle_distance > 0.05
        translation_pinch_active = thumb_index_distance < translation_pinch_threshold and valid_translation_separation
        
        # Add hysteresis for smoother operation
        if hasattr(self, 'prev_translation_pinch_active'):
            if self.prev_translation_pinch_active and thumb_index_distance < translation_pinch_threshold * 1.2:
                translation_pinch_active = True
        self.prev_translation_pinch_active = translation_pinch_active
        
        # Calculate translation pinch position
        translation_pinch_pos = Point()
        translation_pinch_coords = (0, 0)
        if translation_pinch_active:
            translation_pinch_pos.x = (thumb_tip.x + index_tip.x) / 4  # Scale down for sensitivity
            translation_pinch_pos.y = (thumb_tip.y + index_tip.y) / 4
            translation_pinch_pos.z = (thumb_tip.z + index_tip.z) / 4
            translation_pinch_coords = (int((thumb_x + index_x) / 2), int((thumb_y + index_y) / 2))
            rospy.loginfo(f"Translation pinch at: {translation_pinch_pos.x:.3f}, {translation_pinch_pos.y:.3f}")
            
        # ROTATION PINCH (thumb-middle)
        # For rotation, require middle-ring to be separated
        valid_rotation_separation = middle_ring_distance > 0.05
        rotation_pinch_active = thumb_middle_distance < rotation_pinch_threshold and valid_rotation_separation
        
        # Add hysteresis for smoother operation
        if hasattr(self, 'prev_rotation_pinch_active'):
            if self.prev_rotation_pinch_active and thumb_middle_distance < rotation_pinch_threshold * 1.2:
                rotation_pinch_active = True
        self.prev_rotation_pinch_active = rotation_pinch_active
        
        # Calculate rotation pinch position
        rotation_pinch_pos = Point()
        rotation_pinch_coords = (0, 0)
        if rotation_pinch_active:
            rotation_pinch_pos.x = (thumb_tip.x + middle_tip.x) / 4  # Scale down for sensitivity
            rotation_pinch_pos.y = (thumb_tip.y + middle_tip.y) / 4
            rotation_pinch_pos.z = (thumb_tip.z + middle_tip.z) / 4
            rotation_pinch_coords = (int((thumb_x + middle_x) / 2), int((thumb_y + middle_y) / 2))
            rospy.loginfo(f"Rotation pinch at: {rotation_pinch_pos.x:.3f}, {rotation_pinch_pos.y:.3f}")
            
        return (translation_pinch_active, translation_pinch_pos, translation_pinch_coords, thumb_index_distance,
                rotation_pinch_active, rotation_pinch_pos, rotation_pinch_coords, thumb_middle_distance)
    
    def process_camera(self):
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture image from camera")
                continue
            
            # Convert to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Convert back to BGR for OpenCV
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            
            # Initialize pinch states for this frame
            current_translation_pinch_active = False
            current_translation_pinch_pos = None
            current_rotation_pinch_active = False
            current_rotation_pinch_pos = None
            
            # Draw hand landmarks
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Detect both pinch gestures
                    (translation_pinch_active, translation_pinch_pos, translation_pinch_coords, translation_distance,
                     rotation_pinch_active, rotation_pinch_pos, rotation_pinch_coords, rotation_distance) = self.detect_pinches(hand_landmarks, frame.shape)
                    
                    # Update pinch states
                    current_translation_pinch_active = translation_pinch_active
                    current_translation_pinch_pos = translation_pinch_pos
                    current_rotation_pinch_active = rotation_pinch_active
                    current_rotation_pinch_pos = rotation_pinch_pos
                    
                    # Visualize pinch detection
                    if self.show_debug_window:
                        # Draw circles for pinch positions
                        if translation_pinch_active:
                            cv2.circle(frame, translation_pinch_coords, 10, (0, 255, 0), -1)
                            cv2.putText(frame, "TRANSLATE", (translation_pinch_coords[0]+15, translation_pinch_coords[1]), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                        if rotation_pinch_active:
                            cv2.circle(frame, rotation_pinch_coords, 10, (255, 0, 255), -1)
                            cv2.putText(frame, "ROTATE", (rotation_pinch_coords[0]+15, rotation_pinch_coords[1]), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                        
                        # Draw lines between fingers
                        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                        h, w, _ = frame.shape
                        thumb_x, thumb_y = int(thumb_tip.x * w), int(thumb_tip.y * h)
                        index_x, index_y = int(index_tip.x * w), int(index_tip.y * h)
                        middle_x, middle_y = int(middle_tip.x * w), int(middle_tip.y * h)
                        
                        cv2.line(frame, (thumb_x, thumb_y), (index_x, index_y), (0, 255, 0), 2)
                        cv2.line(frame, (thumb_x, thumb_y), (middle_x, middle_y), (255, 0, 255), 2)
                        
                        # Display distances
                        cv2.putText(frame, f"Trans Dist: {translation_distance:.3f}", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame, f"Rot Dist: {rotation_distance:.3f}", (10, 60), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        
                        # Display pinch states
                        trans_state = "Translation: ON" if translation_pinch_active else "Translation: OFF"
                        rot_state = "Rotation: ON" if rotation_pinch_active else "Rotation: OFF"
                        cv2.putText(frame, trans_state, (10, 90), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        cv2.putText(frame, rot_state, (10, 120), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish translation pinch state
            self.pinch_active_pub.publish(Bool(current_translation_pinch_active))
            if current_translation_pinch_active and current_translation_pinch_pos is not None:
                self.pinch_position_pub.publish(current_translation_pinch_pos)
            
            # Publish rotation pinch state
            self.rotation_pinch_active_pub.publish(Bool(current_rotation_pinch_active))
            if current_rotation_pinch_active and current_rotation_pinch_pos is not None:
                self.rotation_pinch_position_pub.publish(current_rotation_pinch_pos)
            
            # Update previous pinch states
            self.pinch_active = current_translation_pinch_active
            self.rotation_pinch_active = current_rotation_pinch_active
            
            # Publish the camera image
            ros_image = self.numpy_to_image_msg(frame)
            self.image_pub.publish(ros_image)
            
            # Show debug window
            if self.show_debug_window:
                cv2.imshow('Hand Tracking', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
        
        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()

if __name__ == '__main__':
    try:
        PinchGestureDetector()
    except rospy.ROSInterruptException:
        pass