# DexHand-HandTracking-ROS2 - Hand Tracking and Joint Angle Publishing via ROS2
# Using MediaPipe Hand Tracking to control the DexHand Simulation
#
# Author: Trent Shumay - trent@iotdesignshop.com

import cv2
import time
import mediapipe as mp
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('handcontroller')
        self.publisher = self.create_publisher(JointState, '/joint_states', 20)
        
        # Constants and Controls
        self.NUM_DOFS = 24       # Number of Degrees of Freedom (DOFs)
        self.JOINT_DEADBAND = 2  # Degrees to ignore for joint movement to help settle noise from MediaPipe
        
        # Debug Drawing Constants
        self.MARGIN = 10  # pixels
        self.FONT_SIZE = 0.5
        self.FONT_THICKNESS = 1
        self.HANDEDNESS_TEXT_COLOR = (88, 205, 54)  # Vibrant green
        
        # Toggles
        self.wrist_enabled = True
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize MediaPipe Hands with appropriate parameters
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        # Open the webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            return
        
        self.get_logger().info("Starting hand tracking. Press 'Esc' to exit, 'w' to toggle wrist.")

        # To handle shutdown
        self._shutdown = False
        
        # Start hand tracking in a separate thread
        self.hand_tracking_thread = threading.Thread(target=self.hand_tracking)
        self.hand_tracking_thread.start()

    def normalize_landmarks(self, joint_xyz):
        """Normalize landmarks relative to the wrist and scale by the hand size."""
        # Use wrist (landmark 0) as the origin
        wrist_position = joint_xyz[0]
        joint_xyz = joint_xyz - wrist_position

        # Scale by the distance between wrist and middle finger base
        scale_factor = np.linalg.norm(joint_xyz[9] - wrist_position)  # Middle finger base
        if scale_factor <= 0.01:
            scale_factor = 1.0  # Avoid division by zero
            self.get_logger().warn("Invalid scale factor detected. Using default value of 1.0.")
            
        joint_xyz = joint_xyz / scale_factor
        return joint_xyz
        

    def angle_between(self, p1, midpt, p2, plane=np.array([1, 1, 1])):
        """Computes the angle between two vectors formed by three points, projected onto a plane."""
        # Compute vectors
        ba = (p1 - midpt) * plane
        bc = (p2 - midpt) * plane

        # Compute norms and handle zero vectors
        norm_ba = np.linalg.norm(ba)
        norm_bc = np.linalg.norm(bc)
        if norm_ba == 0 or norm_bc == 0:
            # If one of the vectors is zero, return 0 degrees (or a default angle)
            return 0.0

        # Normalize vectors
        ba /= norm_ba
        bc /= norm_bc

        # Compute the cosine of the angle
        cosine_angle = np.dot(ba, bc)
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)  # Clamp to avoid numerical errors

        # Compute and return the angle in degrees
        angle = np.arccos(cosine_angle)
        return np.degrees(angle)

    def analyze_hand_landmarks(self, hand_landmarks):
        """Analyze the hand landmarks and map the joint angles to the correct indices."""
        # Assuming only one hand is detected
        landmarks = hand_landmarks.landmark

        # Convert the hand landmark data into a numpy array of joint positions
        joint_xyz = np.array([[lm.x, lm.y, lm.z] for lm in landmarks])

         # Normalize landmarks and analyze joint angles
        joint_xyz = self.normalize_landmarks(joint_xyz)

        # Adjust the size of joint_angles to handle 24 joints
        joint_angles = np.zeros(self.NUM_DOFS)

        # Initialize smoothing and previous angles if not already done
        if not hasattr(self, "smoothed_joint_angles"):
            self.smoothed_joint_angles = np.zeros(self.NUM_DOFS)
        if not hasattr(self, "previous_joint_angles"):
            self.previous_joint_angles = np.zeros(self.NUM_DOFS)

        # Wrist
        if self.wrist_enabled:
            joint_angles[0] = (90 -self.angle_between(
                joint_xyz[13], joint_xyz[0], joint_xyz[0] + np.array([0, 0, 1]), plane=np.array([0, 1, 1]))
            )  # wrist_pitch_lower
            # joint_angles[0] = 0
            # print(joint_angles[0])
            joint_angles[1] = (90 - self.angle_between(
                joint_xyz[13], joint_xyz[0], joint_xyz[0] + np.array([1, 0, 0]), plane=np.array([1, 1, 0]))
            )  # wrist_pitch_upper
            # joint_angles[1] = 0
            # print(joint_angles[1])
            joint_angles[2] = (-0.4*self.angle_between(
                joint_xyz[0], joint_xyz[5], joint_xyz[17], plane=np.array([1,0, 1]))
            )  # wrist_yaw
            # joint_angles[2] = 0
            # print(joint_angles[2])
        else:
            joint_angles[0:3] = 0

        # Index Finger
        joint_angles[3] = (180 - self.angle_between(joint_xyz[0], joint_xyz[5], joint_xyz[6]))  # index_yaw
        joint_angles[7] = (180 - self.angle_between(joint_xyz[5], joint_xyz[6], joint_xyz[7]))  # index_pitch
        # print(joint_angles[7])
        joint_angles[8] = (180 - self.angle_between(joint_xyz[6], joint_xyz[7], joint_xyz[8]))       # index_knuckle
        joint_angles[9] = (self.angle_between(joint_xyz[7], joint_xyz[8], joint_xyz[6]))  # index_tip

        # Middle Finger
        joint_angles[4] = (180 - self.angle_between(joint_xyz[0], joint_xyz[9], joint_xyz[10]))  # middle_yaw
        joint_angles[10] = (180 - self.angle_between(joint_xyz[9], joint_xyz[10], joint_xyz[11]))  # middle_pitch
        joint_angles[11] = (180 - self.angle_between(joint_xyz[10], joint_xyz[11], joint_xyz[12]))      # middle_knuckle
        joint_angles[12] = (self.angle_between(joint_xyz[11], joint_xyz[12], joint_xyz[10]))# middle_tip

        # Ring Finger
        joint_angles[5] = (180 - self.angle_between(joint_xyz[0], joint_xyz[13], joint_xyz[14]))  # ring_yaw
        joint_angles[13] = (180 - self.angle_between(joint_xyz[13], joint_xyz[14], joint_xyz[15])) # ring_pitch
        joint_angles[14] = (180 - self.angle_between(joint_xyz[14], joint_xyz[15], joint_xyz[16]))       # ring_knuckle
        joint_angles[15] = (self.angle_between(joint_xyz[15], joint_xyz[16], joint_xyz[14]))     # ring_tip

        # Pinky Finger
        joint_angles[6] = (180 - self.angle_between(joint_xyz[0], joint_xyz[17], joint_xyz[18]))  # pinky_yaw
        joint_angles[16] = (180 - self.angle_between(joint_xyz[17], joint_xyz[18], joint_xyz[19]))  # pinky_pitch
        joint_angles[17] = (self.angle_between(joint_xyz[18], joint_xyz[19], joint_xyz[17]))       # pinky_knuckle
        joint_angles[18] = (self.angle_between(joint_xyz[19], joint_xyz[20], joint_xyz[18]))      # pinky_tip

        # Thumb Yaw: Angle between thumb base and middle of thumb, relative to the horizontal plane
        joint_angles[19] = self.angle_between(
            joint_xyz[1], joint_xyz[2], joint_xyz[3], plane=np.array([1, 0, 1])
        )  # thumb_yaw

        # Thumb Roll: Angle between thumb base and thumb knuckle, relative to a vertical axis
        joint_angles[20] = self.angle_between(
            joint_xyz[1], joint_xyz[2], joint_xyz[3], plane=np.array([0, 1, 1])
        )  # thumb_roll

        # Thumb Pitch: Angle between thumb segments along its length
        joint_angles[21] = self.angle_between(
            joint_xyz[2], joint_xyz[3], joint_xyz[4], plane=np.array([0, 1, 1])
        )  # thumb_pitch

        # Thumb Knuckle: Angle at the base of the thumb tip
        joint_angles[22] = self.angle_between(
            joint_xyz[3], joint_xyz[4], joint_xyz[4] + np.array([0, 0, 1]), plane=np.array([1, 1, 0])
        )  # thumb_knuckle

        # Thumb Tip: Orientation of the thumb tip relative to its final segment
        joint_angles[23] = self.angle_between(
            joint_xyz[4] - joint_xyz[3], joint_xyz[4], joint_xyz[4] + np.array([0, 0, 1])
        )  # thumb_tip

        # Apply Deadband and Smoothing for Each Joint
        alpha = 0.7  # Smoothing factor
        for i in range(self.NUM_DOFS):
            if abs(joint_angles[i] - self.previous_joint_angles[i]) < self.JOINT_DEADBAND:
                joint_angles[i] = self.previous_joint_angles[i]  # Ignore small changes (deadband)
            self.smoothed_joint_angles[i] = (
                alpha * self.smoothed_joint_angles[i] + (1 - alpha) * joint_angles[i]
            )

        # Save current angles for the next iteration
        self.previous_joint_angles = joint_angles.copy()

        return self.smoothed_joint_angles



    def draw_angles_on_image(self, image, joint_angles):
        """Draw the joint angles on the image for debugging purposes."""
        height, width, _ = image.shape

        # Draw the finger angles
        for i in range(self.NUM_DOFS):
            angle_text = f"{i}: {int(joint_angles[i])}"
            x_pos = int(width * 0.05 + (i // 3) * width * 0.1)
            y_pos = int(height * 0.1 + (i % 3) * height * 0.05)
            cv2.putText(image, angle_text,
                        (x_pos, y_pos), cv2.FONT_HERSHEY_DUPLEX,
                        self.FONT_SIZE, (0,0,0), self.FONT_THICKNESS, cv2.LINE_AA)

        # Draw titles for the columns
        titles = ["idx", "mid", "rng", "pnk", "thb", "wri"]
        for idx, title in enumerate(titles):
            x_pos = int(width * 0.05 + idx * width * 0.1)
            y_pos = int(height * 0.05)
            cv2.putText(image, title, (x_pos, y_pos), cv2.FONT_HERSHEY_DUPLEX,
                        self.FONT_SIZE, (0,0,0), self.FONT_THICKNESS, cv2.LINE_AA)

        return image

    def hand_tracking(self):
        """Hand tracking loop running in a separate thread."""
        rate = 30  # Target publish rate (e.g., 30 Hz)
        publish_interval = 1.0 / rate
        last_publish_time = time.time()

        while not self._shutdown:
            success, frame = self.cap.read()
            if not success:
                self.get_logger().error("Failed to read frame from webcam. Exiting hand tracking loop.")
                break

            # Flip the image horizontally for a mirror view
            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process the frame and detect hands
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    
                    # Draw hand landmarks
                    self.mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,  # Connections to draw lines
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                )

                    # Calculate joint angles
                    joint_angles = self.analyze_hand_landmarks(hand_landmarks)

                    # Draw joint angles and landmarks on the image
                    frame = self.draw_angles_on_image(frame, joint_angles)

                    # Publish only if enough time has passed
                    current_time = time.time()
                    if current_time - last_publish_time >= publish_interval:
                        joint_angles_rad = np.deg2rad(joint_angles)  # Convert to radians if required
                        self.publish_joint_states(joint_angles_rad)
                        last_publish_time = current_time

            # Display the annotated image
            cv2.imshow('Hand Tracking', frame)

            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # Esc key
                self.get_logger().info("Exiting hand tracking loop.")
                self._shutdown = True
                break


    def publish_joint_states(self, joint_angles):
        """Publish joint angles to the '/joint_states' topic."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define joint names - ensure this matches the number of joint_angles
        msg.name = [
            'wrist_pitch_lower',
            'wrist_pitch_upper',
            'wrist_yaw',
            'index_yaw',
            'middle_yaw',
            'ring_yaw',
            'pinky_yaw',
            'index_pitch',
            'index_knuckle',
            'index_tip',
            'middle_pitch',
            'middle_knuckle',
            'middle_tip',
            'ring_pitch',
            'ring_knuckle',
            'ring_tip',
            'pinky_pitch',
            'pinky_knuckle',
            'pinky_tip',
            'thumb_yaw',
            'thumb_roll',
            'thumb_pitch',
            'thumb_knuckle',
            'thumb_tip'
                ]
# 17
        

        # If joint_angles length is different, adjust accordingly
        if len(joint_angles) != len(msg.name):
            self.get_logger().warn(f"Number of joint angles ({len(joint_angles)}) does not match number of joint names ({len(msg.name)}).")
            # Adjust by trimming or padding with zeros
            if len(joint_angles) > len(msg.name):
                joint_angles = joint_angles[:len(msg.name)]
            else:
                joint_angles = np.pad(joint_angles, (0, len(msg.name) - len(joint_angles)), 'constant')

        # Validate joint angles before publishing
        if not np.all(np.isfinite(joint_angles)):
            self.get_logger().error("Invalid joint angles detected before publishing!")
            joint_angles = np.zeros(len(msg.name))  # Reset to a safe state

        # Set the positions
        msg.position = joint_angles.tolist()
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing joint states')
        
            


    def destroy_node(self):
        """Override destroy_node to ensure hand tracking thread is stopped."""
        self._shutdown = True
        if self.hand_tracking_thread.is_alive():
            self.hand_tracking_thread.join()
        self.hands.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
