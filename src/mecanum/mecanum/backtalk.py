import sys
import threading
from threading import Thread, Event
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_srvs.srv import Empty
from std_msgs.msg import String


class ServiceNode(Node):
    def __init__(self):
        super().__init__('mock_service_node')
        self.srv = self.create_service(Empty, 'test_service', callback=self.service_callback)

    def service_callback(self, request, result):
        self.get_logger().info('Server received request')
        return result

class CallbackGroupDemo(Node):
    def __init__(self, client_cb_croup, timer_cb_group, manual_calls):
        super().__init__('callback_group_demo_node')
        self.client = self.create_client(Empty, 'test_service', callback_group=client_cb_croup)
        if not manual_calls:
            self.call_timer = self.create_timer(1, self.timer_cb, callback_group=timer_cb_group)

    def call_srv(self, delay: float = 1):
        sleep(delay)
        self._call_srv()

    def _call_srv(self):
        self.get_logger().info('Client sending request')
        _ = self.client.call(Empty.Request())
        self.get_logger().info('Client received response')

    def timer_cb(self):
        self._call_srv()

def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

def call_srv_manually(client_node):
    client_node.call_srv()
    client_node.get_logger().info('Test finished successfully.\n')

def run_test(client_cb_group, timer_cb_group, manual_call):
    rclpy.init()

    node = CallbackGroupDemo(client_cb_croup=client_cb_group, timer_cb_group=timer_cb_group, manual_calls=manual_call)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    service_node = ServiceNode()
    srv_executor = SingleThreadedExecutor()
    srv_executor.add_node(service_node)
    srv_thread = Thread(target=spin_srv, args=(srv_executor, ), daemon=True)
    srv_thread.start()

    call_thread = Thread(target=call_srv_manually, args=(node, ), daemon=True)
    if manual_call:
        call_thread.start()

    try:
        print("")
        node.get_logger().info('Beginning demo, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    node.destroy_node()
    service_node.destroy_node()
    rclpy.shutdown()
    try:
        srv_thread.join()
    except KeyboardInterrupt:
        pass    
    if manual_call: 
        call_thread.join()

def main(args = None):
    run_test(client_cb_group=MutuallyExclusiveCallbackGroup(), timer_cb_group=None, manual_call=False) 

    run_test(client_cb_group=None, timer_cb_group=MutuallyExclusiveCallbackGroup(), manual_call=False)  

    group1 = MutuallyExclusiveCallbackGroup()
    group2 = MutuallyExclusiveCallbackGroup()
    run_test(client_cb_group=group1, timer_cb_group=group2, manual_call=False)

    cb_group = ReentrantCallbackGroup()

    run_test(client_cb_group=cb_group, timer_cb_group=cb_group, manual_call=False)

if __name__ == '__main__':
    main()