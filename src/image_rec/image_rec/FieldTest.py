import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agrobot_msgs.msg import BoundingBox2DArray, WeedBoundingBox
from vision_msgs.msg import BoundingBox2D
import math
import pyfirmata
import time

class FieldTest(Node):

    def __init__(self):
        super().__init__('fieldtest')
        self.get_logger().info("Field Test Node has started")
        self.bounding_boxes = {}
        self.subscription = self.create_subscription(
            BoundingBox2DArray,
            # 'image_rec/frcnn_prediction', # change here for the real test
            'image_rec/dummy_generator', # dummy test
            self.field_test_callback,
            10)

        self.board = pyfirmata.Arduino("/dev/ttyACM0")
        '''INFO FOR SYNTAX 
            It treates the Arduino board as an object called BOARD and has functions that can be used with it.
            INTILIAZING PINS: object_at_pin = board.digital[PIN_NUMBER]
            WRITING TO A PIN: object_at_pin.write(ON/OFF)
        '''
        self.spraytime = 1 #the time for which the sprayer should spray
        self.ON = 1
        self.OFF = 0
        self.no_of_valves = 9

        #defining some lists/arrays that we use
        self.pin_nos = [3,4,5,6,7,8,9,10,11] # information about the pins that are connected
        self.valve_array = [] #will link the valve number to the pin that it corresponds to          Ex: [VALVE1,VALVE2....] and VALVE1 is set to pin 3 

        #setting up pin info for all the valves
        for i in range(self.no_of_valves):
            valve_name = 'VALVE'+str(i+1)
            valve_name = self.board.digital[self.pin_nos[i]]
            self.valve_array.append(valve_name)

        # conversion from image to spayers
        self.CONVERSION_FACTOR = 1                                                                               # conversion factor for distance in real to image
        self.SPRAY_RADIUS = 1*self.CONVERSION_FACTOR                                                                  # the radius of the spray on ground level in (coord units)
        self.NUM_SPRAYERS = 9
        self.SPRAYER_CENTER_Y = 1                                                                                # the array of sprayer's center's coord value in the y 
        self.SPRAYER_CENTER_X = 1                                                                                # the array of sprayer's center's coord value in the x 
        self.SPRAYER_SEPARATION = 1*self.CONVERSION_FACTOR          
        self.y_lower_thresh = 100
        self.y_upper_thresh = 250                                                  # average separation distance between the sprayer nozzles in (coord units)
        # self.THRESHOLD = SPRAYER_CENTER_Y + SPRAY_RADIUS/2 + 0                                                   # change 0 to adjust the detecting threshold at which to spray
    

    def field_test_callback(self, msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        weed_bounding_boxes = []

       
        str_received = self.determine_sprayers(msg.boxes)


        # str_received = "01100000"   #the string of 0/1 we receive to know which valve to turn on
        
        self.turn_on_valve(str_received,self.spraytime)    

        self.get_logger().info("turning on valves "+str_received)

    def determine_sprayers(self, boxes):
        self.get_logger().info("calculating sprayers")
     
        activated_sprayers = [0]*self.no_of_valves

        for box in boxes:
            center_x = box.center.x
            center_y = box.center.y
            area = (box.size_x) * (box.size_y)

            if center_y > self.y_lower_thresh and center_y < self.y_upper_thresh:
                bin_width = 512/9

                activation_index = center_x//bin_width
                activated_sprayers[int(activation_index)] = 1

        return "".join(str(i) for i in activated_sprayers)
         
                


    
    ''' This code is meant to test the valves of the Agrobot to ensure that the correct valve is turned on based on the string received from Imagerec
        This utilizes the python package "PYFIRMATA()" to communicate with the arduino and uses values of 0 (OFF) or 1 (ON) to represent the valves
        
        Author: Rajalakshmi Narasimhan (rajalakshmi.nr6@gmail.com)
    '''

    ''' FUNCTION: Reset()
        PARAMETERS: none
        OBJECTIVE: to set the intial state - state where all the valves are off
        RETURNS: none 
    '''
    def reset(self):
        for valve in self.valve_array:
            valve.write(self.OFF)

    ''' FUNCTION: all_valves()
        PARAMETERS: spraytime - Time for which it sprays
        OBJECTIVE: To spray all the valves in sequence, without receiving an string
        RETURNS: none 
    '''
    def all_valves(self, spraytime):
        valve_string = "000000000"
        for index in range(len(valve_string)):            
            self.valve_array[index].write(self.ON)
            time.sleep(spraytime)       # IN ARDUINO: delay(time)
            self.valve_array[index].write(self.OFF)
            time.sleep(self.delaytime)

    ''' FUNCTION: turn_on_valve()
        PARAMETERS: string_received - It is a string of 0/1 which indicates which valve should be turned on
                    spraytime - Time for which it sprays
                    delaytime - Time for which it turns off before spraying again
        OBJECTIVE: To spray the correct valve for a fixed time
        RETURNS: none 
    '''
    def turn_on_valve(self, string_received, spraytime):
        for index in range(len(string_received)):
            valve_status = string_received[index]
            if valve_status == "1":
                self.valve_array[index].write(self.ON)
        time.sleep(self.spraytime)       # IN ARDUINO: delay(time)

        for index in range(len(string_received)):
            valve_status = string_received[index]
            if valve_status == "1":
                self.valve_array[index].write(self.OFF)
        time.sleep(self.spraytime)

    ''' FUNCTION: pulse_valve():
        PARAMETERS: string_received - It is an string of 0/1 which indicates which valve should be turned on
                    intial_spraytime - The start time for which the valve sprays
                    min_spraytime - The minimum spray time
                    percentReduction - The percentage by which thespray time decreases every iteration
                    no_of_pulses - no of pulses for each spray time 
        OBJECTIVE: Pulses specified valve on and off for a given number of times and then reduces the spray time by given percentage
    '''
    def pulse_valve(self, string_received, intial_spraytime=3, min_spraytime=1, percentReduction=25, no_of_pulses=3): #with arbitary default values
        spraytime = intial_spraytime
        while spraytime >= min_spraytime:
            for j in range(no_of_pulses):
                self.turn_on_valve(string_received,spraytime)
        spraytime -= spraytime*(percentReduction/100)

    ''' Adapted from Gus' code
        FUNCTION: maxvalvestest():
        PARAMETERS: spraytime - The start time for which the valve sprays
        OBJECTIVE: Turns increasing number of valves on and then off
    '''
    def maxvalvestest(self, valve_array, spraytime):
        range_of_valve = []
        for v in range(len(valve_array)): 
            range_of_valve.append(v)
            self.turn_on_valve(range_of_valve)           



def main(args=None):
    rclpy.init(args=args)
    node = FieldTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
