#!/usr/bin/env python3
########################################################################################################
# Name:             voice_asr.py
# Purpose:          automatic speech recognition module
# Description:      activated by the set wakeword, the voice commands are converted to text and the msg
#                   is published on the voice_cmd topic
# Related Files:    voice_cmd and voice_tts
# Author:           logan naidoo, south africa, 2024
########################################################################################################
# To run this application install the following packages from the terminal command line
# pip3 install pyttsx3
# pip3 install SpeechRecognition
# pip3 install nltk
#     for nltk , run ln_nltk_download.py to download all the dependencies needed for nltk
# pip3 install pygame 
# pip3 install edge-tts

# ## needed for the voice_mov.py module
# sudo apt install ros-humble-nav2-simple-commander
# sudo apt install ros-humble-tf-transformations
# sudo apt install python3-transforms3d
########################################################################################################


import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nltk.tokenize import word_tokenize

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.publisher_ = self.create_publisher(String, 'voice_cmd', 10)
        self.esp_led_publisher_ = self.create_publisher(String, 'esp_led', 10)
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

    def send_esp_led_msg(self, text):
        msg = String()
        msg.data = text
        self.esp_led_publisher_.publish(msg)

    def callback(self):
        with self.microphone_ as source:
            self.recognizer_.adjust_for_ambient_noise(source)
            self.get_logger().info("Listening ... for voice commands")
            self.send_esp_led_msg("listen")  #turn esp32 led on whilst Jupiter is listening
            audio = self.recognizer_.listen(source)
            try:
                self.get_logger().info("Recognizing  ... convert to text")
                self.send_esp_led_msg("recognize")  #turn esp32 led off whilst Jupiter is recognising speech          
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
