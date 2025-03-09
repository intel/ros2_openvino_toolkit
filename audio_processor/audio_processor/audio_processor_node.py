import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ffmpeg
import numpy as np
from pydub import AudioSegment
from openvino.runtime import Core

class AudioProcessorNode(Node):
    def __init__(self):
        super().__init__('audio_processor_node')
        self.publisher_ = self.create_publisher(String, 'stt_output', 10)
        self.ie = Core()
        # Load the converted OpenVINO model
        # self.model = self.ie.read_model(model='wav2vec2-base/wav2vec2-base.xml')
        self.model = self.ie.read_model(model='/root/ros2_ws/audio_processor/audio_processor/wav2vec2-base/wav2vec2-base.xml')
        self.compiled_model = self.ie.compile_model(model=self.model, device_name='CPU')
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)

    def process_audio_file(self, file_path):
        if file_path.endswith('.mp4'):
            audio_data = self.extract_audio_from_mp4(file_path)
        elif file_path.endswith('.wav'):
            audio_data = self.read_wav_file(file_path)
        else:
            self.get_logger().error('Unsupported file format')
            return

        self.process_audio(audio_data)

    def extract_audio_from_mp4(self, file_path):
        audio_output = 'temp_audio.wav'
        ffmpeg.input(file_path).output(audio_output, ac=1, ar='16000').run(overwrite_output=True)
        return self.read_wav_file(audio_output)

    def read_wav_file(self, file_path):
        audio = AudioSegment.from_wav(file_path)
        samples = np.array(audio.get_array_of_samples())
        return samples

    def process_audio(self, audio_data):
        # Preprocess audio_data as needed for your model
        input_data = self.preprocess_audio(audio_data)
        result = self.compiled_model([input_data])[self.output_layer]
        text_output = self.postprocess_result(result)
        self.publish_text(text_output)

    def preprocess_audio(self, audio_data):
        # Normalize audio data
        audio_data = audio_data / np.max(np.abs(audio_data))
    
        # Resample or trim/pad the audio data to 16000 samples
        target_length = 16000
        if len(audio_data) > target_length:
            audio_data = audio_data[:target_length]  # Trim
        else:
            audio_data = np.pad(audio_data, (0, max(0, target_length - len(audio_data))), 'constant')  # Pad

        return np.expand_dims(audio_data, axis=0)  # Add batch dimension

    def postprocess_result(self, result):
        # Implement postprocessing logic to convert model output to text
        return "example text"

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessorNode()
    # Example: Process an audio file
    node.process_audio_file('/root/ros2_ws/audio_processor/audio_processor/1089-134686-0001.wav')
    # node.process_audio_file('1089-134686-0001.wav')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
