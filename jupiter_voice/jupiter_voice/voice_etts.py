#!/usr/bin/env python3
########################################################################################################
# Name:             voice_etts.py
# Purpose:          text to speech module
# Description:      converts text on voice_tts topic to speech audible on the robots audio system
# Related Files:    voice_asr, voice_cmd and voice_tts
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import edge_tts
import pygame
import asyncio
import os

voice_etts=os.environ.get("VOICE_ETTS")     # load the voice male/female type from .bashrc export variable

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/voice_tts',
            self.tts_callback,
            10)

        self.subscription  # Prevent unused variable warning

        # Publisher to notify if audio is playing
        self.audio_playback_pub = self.create_publisher(Bool, '/audio_playback', 10)

        # Initialize pygame mixer
        pygame.mixer.init()

    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Received: "{text}", converting to speech...')
        
        # Use edge-tts to convert text to speech asynchronously
        asyncio.run(self.synthesize_and_play(text))

    async def synthesize_and_play(self, text):
        # Use edge-tts to generate speech (choose a male voice here)
        # tts = edge_tts.Communicate(text, voice="en-US-GuyNeural")  # Male voice
        tts = edge_tts.Communicate(text, voice=voice_etts)  # Male voice
        output_file = "output.mp3"
        await tts.save(output_file)

        # Notify that audio is playing
        self.audio_playback_pub.publish(Bool(data=True))
        
        # Play the generated speech
        self.play_audio(output_file)

        # Notify that audio has finished
        self.audio_playback_pub.publish(Bool(data=False))

    def play_audio(self, filename):
        # Load and play the audio file using pygame
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
