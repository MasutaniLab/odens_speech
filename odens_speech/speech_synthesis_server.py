import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from gtts import gTTS
from io import BytesIO
from mpg123 import Mpg123, Out123


class SpeechSynthesisServer(Node):
    def __init__(self):
        super().__init__('speech_synthesis_server')
        self.get_logger().info('音声合成サーバを起動します．')
        self.lang = 'ja-JP'
        self.out = Out123()
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'speech_synthesis/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の発話を中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('実行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            if goal_handle.request.command != '':
                text = goal_handle.request.command
                self.get_logger().info(f'発話： {text}')
                tts = gTTS(text, lang=self.lang[:2])
                fp = BytesIO()
                tts.write_to_fp(fp)
                fp.seek(0)
                mp3 = Mpg123()
                mp3.feed(fp.read())
                for frame in mp3.iter_frames(self.out.start):
                    if not goal_handle.is_active:
                        self.get_logger().info('中止')
                        return result

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info('キャンセル')
                        return result

                    self.out.play(frame)

                goal_handle.succeed()
                result.answer = 'OK'
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = SpeechSynthesisServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
