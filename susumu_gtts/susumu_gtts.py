import pydub
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from gtts import gTTS
import pyaudio
import threading
import io
import numpy as np


class SusumuGTTS(Node):
    def __init__(self):
        super().__init__('susumu_gtts')
        self.declare_parameter('lang', 'ja')
        self.declare_parameter('playback_speed', 2)
        self.declare_parameter('output_device', None)  # デフォルトデバイスを使用

        self.language = self.get_parameter('lang').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.output_device = self.get_parameter('output_device').value

        self.pyaudio_instance = pyaudio.PyAudio()

        self.list_host_apis()
        self.list_audio_devices()

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
        self.get_logger().info(
            f"Output Device: {self.output_device if self.output_device is not None else 'Default Device'}")

    def list_host_apis(self):
        p = self.pyaudio_instance
        host_api_count = p.get_host_api_count()
        logger = self.get_logger()
        logger.info(f"利用可能なホストAPIの数: {host_api_count}\n")

        for i in range(host_api_count):
            host_api_info = p.get_host_api_info_by_index(i)
            name = host_api_info.get('name')
            version = host_api_info.get('version')
            device_count = host_api_info.get('deviceCount')
            logger.info(f"Host API {i}:")
            logger.info(f"  名前: {name}")
            logger.info(f"  バージョン: {version}")
            logger.info(f"  デバイス数: {device_count}\n")

        # システムのデフォルトホストAPI
        default_host_api_index = p.get_default_host_api_info().get('index')
        default_host_api_name = p.get_default_host_api_info().get('name')
        logger.info(f"デフォルトホストAPI: {default_host_api_name} (Index: {default_host_api_index})")

    def list_audio_devices(self):
        """利用可能なオーディオデバイスの一覧をログに出力します。"""
        self.get_logger().info("Available Audio Devices:")
        for i in range(self.pyaudio_instance.get_device_count()):
            device_info = self.pyaudio_instance.get_device_info_by_index(i)
            device_name = device_info['name']
            max_input = device_info['maxInputChannels']
            max_output = device_info['maxOutputChannels']
            device_type = []
            if max_input > 0:
                device_type.append('Input')
            if max_output > 0:
                device_type.append('Output')
            device_type_str = '/'.join(device_type) if device_type else 'Unknown'
            self.get_logger().info(f"  ID {i}: {device_name} - {device_type_str}")

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
                audio_segment = audio_segment.speedup(
                    playback_speed=self.playback_speed,
                    chunk_size=50
                )

            samples = np.array(audio_segment.get_array_of_samples())
            if audio_segment.channels == 2:
                samples = samples.reshape((-1, 2))
            else:
                samples = samples.reshape((-1, 1))

            samples = samples.astype(np.float32) / 32768.0

            stream = self.pyaudio_instance.open(
                format=pyaudio.paFloat32,
                channels=audio_segment.channels,
                rate=audio_segment.frame_rate,
                output=True,
                output_device_index=self.output_device if self.output_device is not None else None,
                frames_per_buffer=1024
            )

            self.get_logger().info("Starting playback...")

            for i in range(0, len(samples), 1024):
                if self.stop_playback:
                    stream.stop_stream()
                    stream.close()
                    self.get_logger().info("Playback stopped.")
                    return
                block = samples[i:i + 1024]
                stream.write(block.tobytes())

            stream.stop_stream()
            stream.close()
            self.get_logger().info("Playback finished.")
        except Exception as e:
            self.get_logger().error(f"Error during playback: {e}")

    def dynamic_parameter_callback(self, parameters):
        """動的にパラメータが変更された際のコールバック関数。"""
        for param in parameters:
            if param.name == 'lang' and param.type_ == param.Type.STRING:
                self.language = param.value
                self.get_logger().info(f"Language updated to: {self.language}")
            elif param.name == 'playback_speed' and param.type_ == param.Type.DOUBLE:
                self.playback_speed = param.value
                self.get_logger().info(f"Playback speed updated to: {self.playback_speed}")
            elif param.name == 'output_device' and param.type_ in [param.Type.INTEGER, param.Type.STRING]:
                if param.type_ == param.Type.STRING:
                    device_name = param.value
                    self.output_device = self.get_device_index(device_name)
                    self.get_logger().info(f"Output device updated to: {device_name} (ID: {self.output_device})")
                elif param.type_ == param.Type.INTEGER:
                    self.output_device = param.value
                    self.get_logger().info(f"Output device updated to ID: {self.output_device}")
        return SetParametersResult(successful=True)

    def get_device_index(self, device_name):
        for i in range(self.pyaudio_instance.get_device_count()):
            device_info = self.pyaudio_instance.get_device_info_by_index(i)
            if device_info['name'] == device_name:
                return i
        self.get_logger().warn(f"Device '{device_name}' not found. Using default device.")
        return None

    def __del__(self):
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()


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
