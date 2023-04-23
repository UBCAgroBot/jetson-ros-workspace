'''
---------------------------------------------------------------------------------------
DESCRIPTION:
This node periodically takes in messages containing information on the
weed's location in the camera's frame in the form of a bounding box. Along with
this, it receives a time stamp and the weed's personal ID. Given these inputs, 
it publishes a string of zeros and ones that is 9 characters long to a topic
that controls the Arduino pins after the appropriate time delay.

SUBSCRIBES to ImageRec bounding box/ID topic
PUBLISHES to Extermination sprayer activation topic
---------------------------------------------------------------------------------------
ASSUMPTIONS:
(0,0) is located at the bottom left corner of the camera image w/ pos_x pointing right
and pos_y pointing up

msg has the following format:
std_msgs/Header header
float64 center_x
float64 center_y
float64 size_x # width
float64 size_y # height
int32 obj_id # unique ID of bounding box object

Extermination's Arduino node accepts the input with the following arrangement and
is wired to activate as per the diagram below:
                            FRONT OF AGROBOT
                                   ^
                                   |
 SPRAYERS:         0   1   2   3   4   5   6   7   8
Example: the msg '000010001' activates sprayer 4 and 8

There is no restriction on the same weed being sent multiple times over the topic.
---------------------------------------------------------------------------------------
TODO:
Distance units are not set/not yet decided on (m) or (cm) or (mm).

Class variables must be replaced with their actual values, currently there are
placeholders. These values would depend on the distance units chosen above.

Factor to convert real world distance to coord distance must be found.

Change the msg_file_names and topic_names placeholders to their actual names
---------------------------------------------------------------------------------------
'''

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from custom.msg import BoundingBox                                                                      # CHANGE: msg_file_names
from std_msgs.msg import Float64                                                                        # CHANGE: msg_file_names (this is for the speed reading from navigation)
from std_msgs.msg import String                                                                         # CHANGE: msg_file_names (this is for the speed reading from navigation)

class ExterminateArray(Node):
    MAX_SPEED = 0.0                                                                                     # TODO: ask navigation for max speed and convert to (coord units/sec)
    CONVERSION_FACTOR = 1                                                                               # conversion factor for distance in real to image
    SPRAY_RADIUS = 1*CONVERSION_FACTOR                                                                  # the radius of the spray on ground level in (coord units)
    NUM_SPRAYERS = 9
    SPRAYER_CENTER_Y = 1                                                                                # the array of sprayer's center's coord value in the y 
    SPRAYER_CENTER_X = 1                                                                                # the array of sprayer's center's coord value in the x 
    SPRAYER_SEPARATION = 1*CONVERSION_FACTOR                                                            # average separation distance between the sprayer nozzles in (coord units)
    THRESHOLD = SPRAYER_CENTER_Y + SPRAY_RADIUS/2 + 0                                                   # change 0 to adjust the detecting threshold at which to spray
    
    # define the linked list structure
    class ListNode:
       def __init__(self, sprayer=None, y=None, weedID=None, data_time=None):
          self.prevval = None
          self.nextval = None
          self.sprayer = sprayer                                                                        # corresponding sprayer to turn on given an x (0-8)
          self.y = y                                                                                    # y coordinate of the weed from image_rec
          self.weedID = weedID                                                                          # ID of the weed from image_rec
          self.data_time = data_time                                                                    # time the image was taken from image_rec
          
    class DLinkedList:
       def __init__(self):
          self.headval = None
    
    def __init__(self):
        super().__init__('exterminate_array')
        self.speed = 0.0                                                                                # IMPORTANT: this needs to be in (coord units/sec)
        self.head = DLinkedList()
        
        self.subscriber1 = self.create_subscription(BoundingBox, '/image_rec/detect', self.bbox_callback, 10)   # CHANGE: msg_name and topic_name
        self.subscriber2 = self.create_subscription(Float64, '/navigation/speed', self.speed_callback, 10)      # CHANGE: msg_name and topic_name
        self.publisher_ = self.create_publisher(String, '/extermination/spray_array', 10)                       # CHANGE: topic_name
        timer_period = (2*SPRAY_RADIUS)/MAX_SPEED  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def speed_callback(self, msg):
        '''
        TODO: must convert the real world units to image pixel units
               ask navigation what units the speed will be published
        '''
        self.speed = msg.data
        
    def bbox_callback(self, msg):
        '''
        Every time it receives a new weed:
        - check if ID is present
        - if ID is present:
            - update information in linked list
        - else
            - add new at the end of the list
        '''
        if msg.center_y <= THRESHOLD:                                                               # filter out values that are already pass the THRESHOLD
            return
        
        current_node = self.head.headval
        while(True):
            if self.head.headval == None:
                current_time = self.get_clock().now()
                msg_time = Time.from_msg(msg.header.stamp)
                delay = (current_time - msg_time).nanoseconds / 1e9 # seconds
                
                x = msg.center_x
                y = delay*self.speed + msg.center_y                                                # need to adjust given data based on delay of processing
                weedID = msg.obj_id
                data_time = current_time
                new_node = ListNode(x=x, y=y, weedID=weedID, data_time=data_time)

                current_node = new_node
                break
        
            if current_node.weedID == msg.obj_id:
                current_time = self.get_clock().now()
                msg_time = Time.from_msg(msg.header.stamp)
                delay = (current_time - msg_time).nanoseconds / 1e9 # seconds
                
                if NUM_SPRAYERS%2 == 0:                                                            # even number of sprayers
                    starting = SPRAYER_CENTER_X - ((NUM_SPRAYERS/2)-1)*SPRAYER_SEPARATION
                else:
                    starting = SPRAYER_CENTER_X - ((NUM_SPRAYERS/2)-1)*SPRAYER_SEPARATION  - SPRAYER_SEPARATION/2
                
                for x in range(NUM_SPRAYERS):
                    if x == 0:
                        if msg.center_x < starting:
                            current_node.sprayer = x
                    elif x == NUM_SPRAYERS:
                        if msg.center_x >= starting + (x-1)*SPRAYER_SEPARATION:
                            current_node.sprayer = x
                    else:
                        if msg.center_x >= starting + (x-1)*SPRAYER_SEPARATION and msg.center_x < starting + (x)*SPRAYER_SEPARATION:
                            current_node.sprayer = x
                current_node.y = delay*self.speed + msg.center_y                                   # need to adjust given data based on delay of processing
                current_node.weedID = msg.obj_id
                current_node.data_time = current_time
                
            if current_node.nextval == None:
                current_time = self.get_clock().now()
                msg_time = Time.from_msg(msg.header.stamp)
                delay = (current_time - msg_time).nanoseconds / 1e9 # seconds
                
                x = msg.center_x
                y = delay*self.speed + msg.center_y                                                # need to adjust given data based on delay of processing
                weedID = msg.obj_id
                data_time = current_time
                new_node = ListNode(x=x, y=y, weedID=weedID, data_time=data_time)

                current_node.nextval = new_node
                new_node.prevval = current_node
                break
            else:
                current_node = current_node.nextval

    def timer_callback(self):
        '''
        Every i seconds:
        - check the speed of the agrobot and update times (depends on i, speed)
        - publish if y is past threshold
        '''
        sprayer_array_to_send = [0] * NUM_SPRAYERS
        
        current_node = self.head.headval
        while(True):
            if current_node == None:
                break
                
            current_time = self.get_clock().now()
            duration = (current_node.data_time - current_time).nanoseconds / 1e9
            current_node.y = duration*self.speed + current_node.y                                   # adjust the y value
            current_node.data_time = current_time                                                   # change time to now because we updated the values
            
            if current_node.y <= THRESHOLD:                                                         # if it is below the threshold value then it should be sprayed
                sprayer_array_to_send[current_node.sprayer] = 1
                # delete the node and rearrange list
                prev_node = current_node.prevval
                next_node = current_node.nextval
                prev_node.nextval = next_node
                if next_node != None:
                    next_node.prevval = prev_node
                current_node = next_node
            else:    
                current_node = current_node.nextval
        
        # convert int array to string 
        message = ''.join(map(str, sprayer_array_to_send))
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()