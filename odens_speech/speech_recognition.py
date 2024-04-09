import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from ctypes import CFUNCTYPE, c_char_p, c_int, c_char_p, c_int, c_char_p, cdll
from speech_recognition import (
    Recognizer, Microphone, UnknownValueError, RequestError, WaitTimeoutError)


# pyaudioの警告表示抑制
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)


class SpeechRecognition(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        self.get_logger().info('音声認識ノードを起動します．')
        self.lang = 'ja-JP'
        self.recognizer = Recognizer()
        self.active = False
        self.srv_start = self.create_service(Empty, 'speech_recognition/start', self.start_callback)
        self.srv_stop = self.create_service(Empty, 'speech_recognition/stop', self.stop_callback)
        self.publisher = self.create_publisher(String, 'speech_recognition/text', 1)
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def start_callback(self, request, response):
        self.active = True
        self.get_logger().info(f'{self.active=}')
        return response

    def stop_callback(self, request, response):
        self.active = False
        self.get_logger().info(f'{self.active=}')
        return response

    def run(self):
        self.running = True
        while self.running:
            if self.active:
                self.recognition()
            else:
                time.sleep(0.1)

    def recognition(self):
        text = ''
        with Microphone() as source:
            self.get_logger().info('音声入力')
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                audio_data = self.recognizer.listen(
                    source, timeout=10.0, phrase_time_limit=10.0)
            except WaitTimeoutError:
                self.get_logger().info('タイムアウト')
                return
        try:
            self.get_logger().info('音声認識')
            text = self.recognizer.recognize_google(
                audio_data, language=self.lang)
        except RequestError:
            self.get_logger().info('API無効')
            return
        except UnknownValueError:
            self.get_logger().info('認識できない')
        msg = String()
        msg.data = text
        self.get_logger().info(f'認識結果： {text}')
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SpeechRecognition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass

    rclpy.try_shutdown()
