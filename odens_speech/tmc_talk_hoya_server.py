import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
import tmc_talk_hoya_py


class TMCTalkHoyaServer(Node):
    def __init__(self):
        super().__init__('tmc_talk_hoya_server')
        self.get_logger().info('音声合成サーバを起動します．')
        self.lang = 'ja-JP'
        self.goal_handle = None
        self.speaker = tmc_talk_hoya_py.VoiceTextSpeaker(voice='haruka')
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
            if goal_handle.request.command == '':
                return result
            text = goal_handle.request.command
            self.get_logger().info(f'発話： {text}')
            duration = self.speaker.speak(text)
            self._end_time = self.get_clock().now() + Duration(nanoseconds=int(duration*10**9))
            self.get_logger().info(f"発話終了まで: {duration} ")
            while True:
                if not goal_handle.is_active:
                    self.get_logger().info('中止')
                    self.speaker.cancel()
                    return result

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.speaker.cancel()
                    self.get_logger().info('キャンセル')
                    return result

                if self.get_clock().now() > self._end_time:
                    break

            goal_handle.succeed()
            result.answer = 'OK'
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = TMCTalkHoyaServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
