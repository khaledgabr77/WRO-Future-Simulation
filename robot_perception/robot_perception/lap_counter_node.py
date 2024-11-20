#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray
from std_msgs.msg import Float64, Bool

class LapCounterNode(Node):
    def __init__(self):
        super().__init__('lap_counter_node')
        
        self.image_height = 480  
        self.forward_exit_height_percentage = 0.05  # Percentage of the image height to identify exit zone
        self.backward_exit_height_percentage = 0.05
        
        # Set the logger level to DEBUG to see all messages
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.detections_msg = DetectionArray()

        # Driving direction. Affect the counting direction
        # 1: Forward. -1 : Backward
        self.current_direction = 1
        self.previous_direction = 1

        self.section_counter = 0
        self.lap_counter = 0
        self.sections_per_lap = 4

        # Initialize states of state machine
        self.START = True
        self.WAITING_FOR_ENTRANCE = False
        self.IN_ENTRANCE = False
        self.EXIT = False

        self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10)
        
        self.create_subscription(
            Float64,
            '/drive',
            self.drive_callback,
            10)
        
        self.lap_counter_publisher = self.create_publisher(Float64, '/lap_counter', 10)
        self.line_entrance_publisher = self.create_publisher(Bool, '/line_entrance', 10)

        
        # State machine loop
        # self.create_timer(0.02, self.state_machine_loop)

    def is_object_in_entrance(self) -> bool:
        '''
        1. check if there is detection
        '''
        found_line = False
        det = None
        line_max_y = -999999
        for detection in self.detections_msg.detections:
            if  detection.class_name == "blue_line":
                y_coordinate = (detection.bbox.center.position.y )
                if y_coordinate  > line_max_y:
                # if (detection.bbox.center.position.y )  > line_max_y:
                    line_max_y = y_coordinate
                    # line_max_y = (detection.bbox.center.position.y )
                    det = detection
                    found_line = True

        if not found_line:
            return False
        
        
        object_y_coordinate =  (det.bbox.center.position.y+ det.bbox.size.y/2)
        if self.current_direction > 0:
            if object_y_coordinate > 0 and object_y_coordinate < int( self.image_height * (1- self.forward_exit_height_percentage) ):
                return True
            else:
                return False
        elif self.current_direction < 0:
            if object_y_coordinate > int( self.image_height * (1- self.backward_exit_height_percentage) ) and object_y_coordinate < self.image_height:
                return True
            else:
                return False

        else: return False
    
    def is_object_in_exit_zone(self) -> bool:
        '''
        1. check if line in exit zone
        '''
        found_line = False
        det = None
        line_max_y = -999999
        for detection in self.detections_msg.detections:
            if detection.class_name == "blue_line":
                y_coordinate = (detection.bbox.center.position.y) 
                if y_coordinate > line_max_y:
                # if (detection.bbox.center.position.y )  > line_max_y:
                    line_max_y = y_coordinate
                    # line_max_y = y_coordinate
                    det = detection
                    found_line = True

        if not found_line:
            return False
        
        object_y_coordinate =  (det.bbox.center.position.y + det.bbox.size.y/2)
        if self.current_direction > 0:
            if object_y_coordinate < self.image_height and object_y_coordinate > int( self.image_height * (1- self.forward_exit_height_percentage) ):
                return True
            else:
                return False
        elif self.current_direction < 0:
            if object_y_coordinate < int( self.image_height * (1- self.backward_exit_height_percentage) ) and object_y_coordinate > 0:
                return True
            else:
                return False

        else: return False

    
    def increase_section_counter(self):
        self.section_counter += 1
        return
    
    def decrease_section_counter(self):
        self.section_counter -= 1
        if self.section_counter < 0:
            self.section_counter = 0
        return
    
    def update_lap_counter(self):
        self.lap_counter = float(self.section_counter // self.sections_per_lap)
        return
    
    def direction_flipped(self) -> bool:
        if self.current_direction != self.previous_direction:
            self.previous_direction = self.current_direction
            return True
        else: return False

    def update_counters(self):
        if self.current_direction >= 0:
            self.increase_section_counter()
            self.update_lap_counter()
        else:
            self.get_logger().warn(f'Decreasing section counter')
            self.decrease_section_counter()
            self.update_lap_counter()
        return
    
    def reset_state_machine(self):
        self.START = False
        self.WAITING_FOR_ENTRANCE = False
        self.IN_ENTRANCE = False
        self.EXIT = False
    
    def state_machine_loop(self):

        # self.is_line_in_entrance()

        if self.START:
            self.START = False
            self.WAITING_FOR_ENTRANCE = True
            # self.get_logger().info(f'Going from START -> WAITING_FOR_ENTRANCE ')

        if self.WAITING_FOR_ENTRANCE:
            # self.get_logger().info(f'In WAITING_FOR_ENTRANCE state')
            if self.direction_flipped():
                self.get_logger().warn(f'Direction is flipped. Resetting state machine')
                self.reset_state_machine()
                self.START = True
                return
            if self.is_object_in_entrance():
                self.reset_state_machine()
                self.IN_ENTRANCE = True
                # self.get_logger().info(f'Going from  WAITING_FOR_ENTRANCE -> IN_ENTRANCE')
                return

        if self.IN_ENTRANCE :
            # self.get_logger().info(f'In IN_ENTRANCE state')
            if self.direction_flipped():
                self.get_logger().warn(f'Direction is flipped. Resetting state machine')
                self.reset_state_machine()
                self.START = True
                return
            
            if self.is_object_in_exit_zone():
                self.reset_state_machine()
                self.EXIT = True
                # self.get_logger().info(f'Going from IN_ENTRANCE -> EXIT')
                return

        if self.EXIT :
            # self.get_logger().info(f'In EXIT state')
            if self.direction_flipped():
                self.get_logger().warn(f'Direction is flipped. Resetting state machine')
                self.reset_state_machine()
                self.START = True
                return
            
            self.update_counters()
            self.get_logger().info(f'Section counter : {self.section_counter}. Lap counter: {self.lap_counter}.')
            self.reset_state_machine()
            self.WAITING_FOR_ENTRANCE = True
            # self.get_logger().info(f'Going from EXIT -> WAITING_FOR_ENTRANCE')
            return

        self.lap_counter_publisher.publish(Float64(data=float(self.lap_counter)))


    def is_line_in_entrance(self):
        found_line = False
        det = None
        line_max_y = -999999
        for detection in self.detections_msg.detections:
            if detection.class_name == "blue_line":
                y_coordinate = (detection.bbox.center.position.y) 
                if y_coordinate > line_max_y:
                # if (detection.bbox.center.position.y )  > line_max_y:
                    line_max_y = y_coordinate
                    # line_max_y = y_coordinate
                    det = detection
                    found_line = True

        if det is None:
            return
        if (det.bbox.center.position.y - det.bbox.size.y/2) > (self.image_height/2+0.1*self.image_height):
            self.get_logger().info(f'line is in entrance')
            self.line_entrance_publisher.publish(Bool(data=True))
        else:
            self.line_entrance_publisher.publish(Bool(data=False))

    
    def drive_callback(self, msg: Float64):
        if msg.data > 0 or msg.data < 0:
            self.current_direction = msg.data
        return

    def detection_callback(self, msg: DetectionArray):
        self.detections_msg = msg
        self.state_machine_loop()
      

def main(args=None):
    rclpy.init(args=args)
    lap_counter = LapCounterNode()
    
    try:
        rclpy.spin(lap_counter)
    except KeyboardInterrupt:
        pass
    finally:
        lap_counter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
