#!/usr/bin/env python3
########################################################################################################
# Name:             voice_wsp.py
# Purpose:          automatic speech recognition module using the WHISPER system from OpenAI
# Description:      activated by the set wakeword, the voice commands are converted to text and the msg
#                   is published on the /voice_cmd topic
#                   /voice_id publishes the state of the voice system which changes gui images
# Related Files:    pip install openai-whisper sounddevice numpy
# Author:           logan naidoo, south africa, 2024
########################################################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import whisper

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Publisher to the /voice_cmd topic
        # self.publisher_ = self.create_publisher(String, '/voice_cmd', 10)
        self.voice_cmd_publisher_ = self.create_publisher(String, 'voice_cmd', 10)      # publishes voice data
        self.voice_id_publisher_ = self.create_publisher(String, 'voice_id', 10)        # publishes voice indicator either 'listen' or 'recognize'
        
        # Load Whisper model
        self.model = whisper.load_model("base")
        
        # Define parameters
        self.sample_rate = 16000  # Whisper uses 16 kHz audio
        self.duration = 10  # Recording duration in seconds

        # Set up a timer to call the function periodically
        self.timer = self.create_timer(10.0, self.timer_callback)

        #setup robot voice wakewords 
        self.wakeword1 = 'jupiter'                      # robot wakeword
        self.wakeword2 = 'tubidy'                       # may sound like robot wakeword
        self.wakeword_tokens = ['jupiter','tubidy']     # add more tokens for similar sounding wake words here
    
    def send_voice_cmd_msg(self, text):
        msg = String()
        text = text.lower().replace(self.wakeword1,"")
        text = text.lower().replace(self.wakeword2,"")  # add below self.wakeword3_ if you add to wake word list in constructor above
        msg.data = text                                 # load the ROS2 String msg with the user command excluding the wake word
        self.voice_cmd_publisher_.publish(msg)          # publish the /voice_cmd message
        # self.get_logger().info(msg.data)

    def send_voice_id_msg(self, text):                  # just packages the voice_id text into String msg to publish on /voice_id topic
        msg = String()
        msg.data = text
        self.voice_id_publisher_.publish(msg)

    def record_audio(self):
        """Records audio from the microphone for a given duration."""
        self.get_logger().info("Recording...")
        self.send_voice_id_msg("listening")            # robot is in 'listen' mode
        audio = sd.rec(int(self.duration * self.sample_rate), 
                       samplerate=self.sample_rate, 
                       channels=1, dtype=np.float32)
        sd.wait()  # Wait until recording is finished
        self.get_logger().info("Recording finished.")
        return np.squeeze(audio)

    def transcribe_audio(self, audio):
        """Transcribes the audio using the Whisper model."""
        self.get_logger().info("Transcribing...")
        self.send_voice_id_msg("recognize")            # robot is in 'listen' mode
        result = self.model.transcribe(audio, fp16=False)
        self.get_logger().info("Transcription complete.")
        return result['text']

    def timer_callback(self):
        """Handles recording audio and publishing the transcribed text."""
        
        audio_data = self.record_audio()
        text_output = self.transcribe_audio(audio_data)
        # check if the wake word list is in the recognized text
        if any(item in text_output.lower() for item in self.wakeword_tokens):
            if len(text_output.split()) >= 3:              # only process audio if 3 or more words
                self.send_voice_cmd_msg(text_output)
        else: 
            # self.get_logger().warn("Wakeword not detected")
            self.send_voice_id_msg("wakeword not detected")
        
        # Publish the recognized text
        # msg = String()
        # msg.data = text_output
        # self.voice_cmd_publisher_.publish(msg)
        # self.get_logger().info(f"Published: {text_output}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
