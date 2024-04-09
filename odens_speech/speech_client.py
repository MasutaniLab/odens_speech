from threading import Thread, Event
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class StringCommandActionClient:
    def __init__(self, node, name):
        self.name = name
        self.logger = node.get_logger()
        self.action_client = ActionClient(node, StringCommand, name)
        self.event = Event()

    def send_goal(self, command: str):
        self.logger.info(f'{self.name} アクションサーバ待機...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.logger.info(f'{self.name} ゴール送信... command: \'{command}\'')
        self.event.clear()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.event.wait(20.0)
        if self.action_result is None:
            self.logger.info(f'{self.name} タイムアウト')
            return None
        else:
            result = self.action_result.result
            status = self.action_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f'{self.name} 結果: {result.answer}')
                self.goal_handle = None
                return result.answer
            else:
                self.logger.info(f'{self.name} 失敗ステータス: {status}')
                return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info(f'{self.name} ゴールは拒否されました')
            return
        self.goal_handle = goal_handle
        self.logger.info(f'{self.name} ゴールは受け付けられました')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.event.set()


class SpeechClient(Node):
    def __init__(self):
        super().__init__('speech_client')
        self.get_logger().info('音声対話ノードを起動します．')
        self.recognition_client = StringCommandActionClient(
            self, 'speech_recognition/command')
        self.synthesis_client = StringCommandActionClient(
            self, 'speech_synthesis/command')
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.running = True
        while self.running:
            text = self.recognition_client.send_goal('')
            if text is not None and text != '':
                self.get_logger().info(f'入力： {text}')
                text2 = text + 'だよね'
                self.get_logger().info(f'出力： {text2}')
                self.synthesis_client.send_goal(text2)



def main():
    rclpy.init()
    node = SpeechClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass

    rclpy.try_shutdown()
