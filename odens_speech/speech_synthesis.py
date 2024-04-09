import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
from io import BytesIO
from mpg123 import Mpg123, Out123


class SpeechSynthesis(Node):
    def __init__(self):
        super().__init__('speech_synthesis')
        self.get_logger().info('音声合成ノードを起動します．')
        self.lang = 'ja-JP'
        self.mp3 = Mpg123()
        self.out = Out123()
        self.subscriber = self.create_subscription(
            String, 'speech_synthesis/text', self.synthesis, 1)

    def synthesis(self, msg):
        if msg.data != '':
            text = msg.data
            self.get_logger().info(f'発話： {text}')
            tts = gTTS(text, lang=self.lang[:2])
            fp = BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)
            self.mp3.feed(fp.read())
            for frame in self.mp3.iter_frames(self.out.start):
                self.out.play(frame)


def main():
    rclpy.init()
    node = SpeechSynthesis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
