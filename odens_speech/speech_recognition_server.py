import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
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


class SpeechRecognitionServer(Node):
    def __init__(self):
        super().__init__('speech_recognition_server')
        self.get_logger().info('音声認識サーバを起動します．')
        self.lang = 'ja-JP'
        self.recognizer = Recognizer()
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'speech_recognition/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の音声入力を中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('実行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            with Microphone() as source:
                self.get_logger().info('音声入力')
                self.recognizer.adjust_for_ambient_noise(source)
                try:
                    audio_data = self.recognizer.listen(
                        source, timeout=10.0, phrase_time_limit=10.0)
                except WaitTimeoutError:
                    self.get_logger().info('タイムアウト')
                    return result

            if not goal_handle.is_active:
                self.get_logger().info('中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('キャンセル')
                return result

            text = ''
            try:
                self.get_logger().info('音声認識')
                text = self.recognizer.recognize_google(
                    audio_data, language=self.lang)
            except RequestError:
                self.get_logger().info('API無効')
                return result

            except UnknownValueError:
                self.get_logger().info('認識できない')

            if not goal_handle.is_active:
                self.get_logger().info('中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('キャンセル')
                return result

            goal_handle.succeed()
            result.answer = text
            self.get_logger().info(f'認識結果： {text}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = SpeechRecognitionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
