#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import os

class TextToSpeech(Node):
    def __init__(self):
        super().__init__('text_to_speech')
        self.subscription = self.create_subscription(String, '/voice_tts', self.speak_callback, 10)
        pygame.mixer.init()
        pygame.mixer.set_num_channels(8)

    def speak_callback(self, msg):
        text = msg.data
        text = text.replace('"','') # remove qoutes "" from text string as it causes error in edge-tts processing
        voice = 'en-US-ChristopherNeural'
        #voice = 'en-GB-SoniaNeural'
        chunks = text.split()
        chunk_size = 100
        chunks = [chunks[i:i+chunk_size]for i in range(0,len(chunks),chunk_size)]

        for chunk in chunks:
            text = ' '.join(chunk)
            #data = f'python -m edge_tts --voice "{voice}" --text "{text}" --write-media "data1.mp3"'
            data = f'edge-tts --voice "{voice}" --text "{text}" --write-media "data.mp3"'
            os.system(data)

            pygame.init()
            pygame.mixer.init()
            pygame.mixer.music.load("data.mp3")

            try:
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
            except Exception as e:
                self.get_logger().error(f'Request error: {e}')
            finally:
                pygame.mixer.music.stop()
                pygame.mixer.quit()
        return True

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeech()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
