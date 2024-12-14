import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import simpleaudio as sa
import threading
import io
import pydub


class SusumuGTTS(Node):
    def __init__(self):
        super().__init__('susumu_gtts')

        # 言語設定用のパラメータを宣言（デフォルトは日本語 'ja'）
        self.declare_parameter('lang', 'ja')
        self.language = self.get_parameter('lang').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            '/tts',
            self.speech_callback,
            10)

        self.playback_thread = None
        self.stop_playback = False
        self.lock = threading.Lock()
        self.get_logger().info(f"Susumu gTTS Node started. Subscribed to /tts topic. Language: {self.language}")

    def speech_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received text: '{text}'")

        # 再生中の音声を停止
        with self.lock:
            if self.playback_thread and self.playback_thread.is_alive():
                self.get_logger().info("Stopping current playback.")
                self.stop_playback = True
                self.playback_thread.join()  # スレッドの終了を待つ
                self.stop_playback = False

        # 新しい音声を再生
        self.playback_thread = threading.Thread(target=self.play_audio, args=(text,))
        self.playback_thread.start()

    def play_audio(self, text):
        """ gTTSを使用してメモリ上で音声を生成し再生 """
        try:
            self.get_logger().info(f"Generating audio with language '{self.language}'...")
            tts = gTTS(text=text, lang=self.language)
            audio_buffer = io.BytesIO()
            tts.write_to_fp(audio_buffer)
            audio_buffer.seek(0)

            # MP3データをWAVに変換して再生
            audio_segment = pydub.AudioSegment.from_file(audio_buffer, format="mp3")
            playback_data = audio_segment.raw_data
            playback_obj = sa.play_buffer(playback_data, num_channels=audio_segment.channels,
                                          bytes_per_sample=audio_segment.sample_width,
                                          sample_rate=audio_segment.frame_rate)

            # 再生監視
            while playback_obj.is_playing():
                if self.stop_playback:
                    playback_obj.stop()
                    self.get_logger().info("Playback stopped.")
                    return
            self.get_logger().info("Playback finished.")
        except Exception as e:
            self.get_logger().error(f"Error during playback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SusumuGTTS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
