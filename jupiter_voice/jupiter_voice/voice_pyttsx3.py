#!/usr/bin/env python3
########################################################################################################
# Name:             voice_pyttsx3.py
# Purpose:          text to speech module
# Description:      converts text on voice_tts topic to speech audible on the robots audio system
# Related Files:    voice_asr, voice_cmd and voice_tts
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Create a subscriber to the /voice_tts topic
        self.subscription = self.create_subscription(
            String,
            '/voice_tts',
            self.tts_callback,
            10
        )

        # Initialize pyttsx3 TTS engine
        self.engine = pyttsx3.init()

        # Optionally, set speech properties (rate, volume)
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 1)

        self.get_logger().info('Text-to-Speech Node has started.')

    def tts_callback(self, msg):
        """Callback function to process received text and convert to speech."""
        self.get_logger().info(f'Received text: "{msg.data}"')

        # Use pyttsx3 to speak the received text
        self.engine.say(msg.data)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the TTS node
    tts_node = TtsNode()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node when done
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
