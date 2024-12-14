import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        # create_service(service_type, service_name, callback)
        # 在 ROS 2 中，'speech_text' 是服务的名称，它并不是提前定义的，而是在代码中通过 create_service 函数显式指定的
        self.speech_service_ = self.create_service(SpeechText, 'speech_text', self.speech_text_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'en'

    def speech_text_callback(self, request, response):
        self.get_logger().info('Preparing to read aloud:{request.text}')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result = True
        return response
    
def main():
    rclpy.init()
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()
