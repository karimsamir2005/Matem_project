import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Joy


ARM_ACTIVATION ='ARM'
BASE_ACTIVATION = 'BASE'

AUTO_ACTIVATION ='AUTO'
MANUAL_ACTIVATION = 'MAN'
BONUS_ACTIVATION = 'BONUS'

IDX_MODULES_SWITCHING=2
IDX_MODE_SWITCHING=9
IDX_BONUS=3  # square button

class ModeManagerNode(Node):

    def __init__(self):
        super().__init__('mode_manager_node')
        #Start in the base
        self.module=BASE_ACTIVATION
        self.mode=MANUAL_ACTIVATION
        # Track previous mode_toggle value to detect rising edges
        self._prev_toggle_module = False
        self._prev_toggle_mode=False
        self._prev_bonus = False
        self.bonus_flag = False

        # Subscribers
        self._joy_sub = self.create_subscription(
            Joy,              # Subscribed message type
            '/joy',               # Subscribed topic name
            self.manager_callback,       # Call back function
            10                              # Buffer
        )           

        # Publisher
        self._base_tele_pub = self.create_publisher(
            Joy,              # Published message type
            '/base_joy',          # Published topic name
            10                              # Buffer
        )
        # Publisher
        self._arm_pub = self.create_publisher(
            Joy,              # Published message type
            '/arm_joy',          # Published topic name
            10                              # Buffer
        )
        # Publish mode so other nodes can read it
        self._module_pub = self.create_publisher(
            String,                         # Published message type
            '/robot_mode',                    # Published topic name    
            10                              # Buffer
        )
        self._mode_pub=self.create_publisher(
            String,
            'auto_active',
            10
        )
        self.timer = self.create_timer(0.05, self.publish_outputs)



        # self.get_logger().debug(
        #     'mode_manager_node Bada2t.\t'
        #     '  Listening : /joy\t'
        #     '  Publishing: /base_tele-op, /arm_ps4_input\t'
        #     f'  Start MODULES: {self.module}\t '
        # )

    # ── Private helpers ──────────────────────────────────────────────────────
    def _is_rising_edge(self, current: bool, previous: bool) -> bool:
        # Return True only on the frame a signal transitions 0 -> 1
        return current and not previous

    def _switch_to_Base(self):
        self.module=BASE_ACTIVATION
       # self.get_logger().info('base mode is activated')

    def _switch_to_Arm(self):
        self.module=ARM_ACTIVATION
       # self.get_logger().info('arm mode is activated')

    def _switch_to_Auto(self):
        self.mode=AUTO_ACTIVATION
      #  self.get_logger().info('autonmous mode is activated')
    
    def _switch_to_man(self):
        self.mode=MANUAL_ACTIVATION
       # self.get_logger().info('manual mode is activated')

    def _switch_to_bonus(self):
        self.mode=BONUS_ACTIVATION
       # self.get_logger().info('bonus mode is activated')


    def _handle_module_toggle(self, toggle_value:bool):
        toggle_now = toggle_value 
        if self._is_rising_edge(toggle_now, self._prev_toggle_module):
            if self.module == BASE_ACTIVATION:
                self._switch_to_Arm()
            else:
                self._switch_to_Base()
        self._prev_toggle_module = toggle_now  # update for next frame

    def _handle_mode_toggle(self, toggle_value:bool):
        toggle_now = toggle_value
        if self._is_rising_edge(toggle_now, self._prev_toggle_mode):
            if self.mode== MANUAL_ACTIVATION:
                self._switch_to_Auto()
            else:
                self._switch_to_man()
        self._prev_toggle_mode = toggle_now  # update for next frame

    def _handle_bonus_toggle(self, toggle_value: bool):
        toggle_now = toggle_value
        if self._is_rising_edge(toggle_now, self._prev_bonus):
            if self.module == BASE_ACTIVATION and self.mode == MANUAL_ACTIVATION:
                self.bonus_flag = not self.bonus_flag
        self._prev_bonus = toggle_now
    # ── Main callbacks ───────────────────────────────────────────────────────

    def manager_callback(self, msg: Joy) -> None:
        
        self._handle_module_toggle(msg.buttons[IDX_MODULES_SWITCHING])
        self._handle_mode_toggle(msg.buttons[IDX_MODE_SWITCHING])
        self._handle_bonus_toggle(msg.buttons[IDX_BONUS])
             
        
        if self.mode == MANUAL_ACTIVATION:
            if self.module == BASE_ACTIVATION:
                #self._switch_to_Base()
                self._base_tele_pub.publish(msg)    
            else: 
                #self._switch_to_Arm()
                self._arm_pub.publish(msg)
        else:
          return
            
            

    def publish_outputs(self):
        msg = String()
        msg.data=self.module
        self._module_pub.publish(msg)
        
        mode=String()
        mode.data = self.mode + ('_BONUS' if self.bonus_flag else '')
        self._mode_pub.publish(mode)
        
       
            
            
    



# ── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()