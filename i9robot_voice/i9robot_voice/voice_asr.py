#!/usr/bin/env python3

import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nltk.tokenize import word_tokenize

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.publisher_ = self.create_publisher(String, 'voice_cmd', 10)
        self.timer_ = self.create_timer(2.0, self.callback)
        self.recognizer_ = sr.Recognizer()
        self.microphone_ = sr.Microphone()

        #setup robot voice wakewords 
        self.wakeword1 = 'jupiter'
        self.wakeword2 = 'tubidy'
        self.wakeword_tokens = ['jupiter','tubidy'] # add more tokens for similar sounding wake words here
        
    def send_voice_command(self, text):
        msg = String()
        text = text.lower().replace(self.wakeword1,"")
        text = text.lower().replace(self.wakeword2,"") # add below self.wakeword3_ if you add to wake word list in constructor above
        msg.data = text # load the ROS2 String msg with the user command excluding the wake word
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

    def callback(self):
        with self.microphone_ as source:
            self.recognizer_.adjust_for_ambient_noise(source)
            self.get_logger().info("Listening ... for voice commands")
            audio = self.recognizer_.listen(source)
            try:
                self.get_logger().info("Recognizing  ... convert to text")            
                text = self.recognizer_.recognize_google(audio)
                self.get_logger().info(text)
                # check if the wake word list is in the recognized text
                if any(item in text.lower() for item in self.wakeword_tokens):
                    if len(text.split()) >= 4: # only process audio if 4 or more words
                        self.send_voice_command(text)
                else: 
                    self.get_logger().warn("Wakeword not detected")
            except sr.UnknownValueError:
                self.get_logger().warn('Unable to recognize speech')
            except sr.RequestError as e:
                self.get_logger().error(f'Request error: {e}')
    
def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
