import pydub
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from gtts import gTTS
import simpleaudio as sa
import threading
import io


class SusumuGTTS(Node):
    def __init__(self):
        super().__init__('susumu_gtts')
        self.declare_parameter('lang', 'ja')
        self.declare_parameter('playback_speed', 1.5)
        self.language = self.get_parameter('lang').value
        self.playback_speed = self.get_parameter('playback_speed').value

        self.subscription = self.create_subscription(
            String,
            '/tts',
            self.speech_callback,
            10)

        self.add_on_set_parameters_callback(self.dynamic_parameter_callback)

        self.playback_thread = None
        self.stop_playback = False
        self.lock = threading.Lock()
        self.get_logger().info(f"Susumu gTTS Node started. Subscribed to /tts topic.")
        self.get_logger().info(f"Language: {self.language}, Playback Speed: {self.playback_speed}x")

    def speech_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received text: '{text}'")

        with self.lock:
            if self.playback_thread and self.playback_thread.is_alive():
                self.get_logger().info("Stopping current playback.")
                self.stop_playback = True
                self.playback_thread.join()
                self.stop_playback = False
        self.playback_thread = threading.Thread(target=self.play_audio, args=(text,))
        self.playback_thread.start()

    def play_audio(self, text):
        try:
            self.get_logger().info(f"Generating audio with language '{self.language}'...")
            tts = gTTS(text=text, lang=self.language)
            audio_buffer = io.BytesIO()
            tts.write_to_fp(audio_buffer)
            audio_buffer.seek(0)

            audio_segment = pydub.AudioSegment.from_file(audio_buffer, format="mp3")
            if self.playback_speed != 1.0:
                audio_segment = audio_segment.speedup(playback_speed=self.playback_speed, chunk_size=50, crossfade=0)
            playback_data = audio_segment.raw_data
            playback_obj = sa.play_buffer(playback_data, num_channels=audio_segment.channels,
                                          bytes_per_sample=audio_segment.sample_width,
                                          sample_rate=audio_segment.frame_rate)
            while playback_obj.is_playing():
                if self.stop_playback:
                    playback_obj.stop()
                    self.get_logger().info("Playback stopped.")
                    return
            self.get_logger().info("Playback finished.")
        except Exception as e:
            self.get_logger().error(f"Error during playback: {e}")

    def dynamic_parameter_callback(self, parameters):
        for param in parameters:
            if param.name == 'lang' and param.type_ == param.Type.STRING:
                self.language = param.value
                self.get_logger().info(f"Language updated to: {self.language}")
            elif param.name == 'playback_speed' and param.type_ == param.Type.DOUBLE:
                self.playback_speed = param.value
                self.get_logger().info(f"Playback speed updated to: {self.playback_speed}")
        return SetParametersResult(successful=True)


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
