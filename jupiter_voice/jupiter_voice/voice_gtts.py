#!/usr/bin/env python3
########################################################################################################
# Name:             voice_gtts.py
# Purpose:          text to speech module using google tts platform
# Description:      converts text on voice_tts topic to speech audible on the robots audio system
# Related Files:    pip install pygame gtts
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import pygame
import os

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/voice_tts',
            self.tts_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Initialize pygame mixer
        pygame.mixer.init()

    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received: "{text}", converting to speech...')
        
        # Convert text to speech and save as an MP3 file
        tts = gTTS(text)
        tts.save('output.mp3')
        
        # Play the generated speech
        self.play_audio('output.mp3')

    def play_audio(self, filename):
        # Load and play the audio file
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        # Wait until the audio has finished playing
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
