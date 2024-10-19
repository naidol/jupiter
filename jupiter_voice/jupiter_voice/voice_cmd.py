#!/usr/bin/env python3
########################################################################################################
# Name:             voice_cmd.py
# Purpose:          receives voice commands on voice_cmd topic and processes it accordingly
# Description:      commands are processed locally or sent to chatGPT. responses are published on the
#                   voice_tts topic where text is converted to speech
# Related Files:    voice_asr and voice_tts
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import openai
import os
import datetime
from jupiter_voice.gpt_memory import GPTMemoryNode  # Import the GPTMemoryNode class

# Set OpenAI API key
# make sure to set your OpenAI account key in bashrc --> export OPENAI_API_KEY <your private key>
# client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"),)

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")
        self.subscription = self.create_subscription(String, "/voice_cmd", self.callback, 10)
        self.publisher = self.create_publisher(String, "/voice_tts", 10)
        self.messages = [] #this array is used to store conversations with GPT AI
        # Create an instance of GPTMemoryNode
        self.gpt_memory_node = GPTMemoryNode()

    # check for specific voice commands, else assume that its a standard voice request to GPT
    def callback(self, msg):       
        prompt = msg.data   # get the voice command message into text format
        if 'time' in prompt.lower():
            time_text = self.calc_time()
            self.send_voice_tts('the time is ' + time_text)
        elif 'date' in prompt.lower():
            date_text = self.calc_date()
            self.send_voice_tts('the date is ' + date_text)
        elif 'move' in prompt.lower():
            name_text='ok. i am moving.'
            self.send_voice_tts(name_text)
        elif 'voice system error' in prompt.lower():
            error_text = 'detecting voice system fault. i will attempt to resolve.'
            self.send_voice_tts(error_text)
        else:
            # Call GPTMemoryNode's method to process the user prompt
            self.gpt_memory_node.process_user_prompt(prompt)
            # gpt_output = self.get_chat_gpt_response(prompt)
            # self.send_voice_tts(gpt_output)
    
    # send the message to convert text to voice via the /voice_tts topic
    def send_voice_tts(self, text):
        tts_msg = String()
        tts_msg.data = text
        self.get_logger().info(text)
        self.publisher.publish(tts_msg)

    def calc_time(self):
        text_time = datetime.datetime.now().strftime("%H:%M:%S") # Hour = I, Min = M, Sec = S
        return text_time
    
    def calc_date(self):
        text_date = datetime.datetime.now().strftime("%d:%B:%Y")
        return text_date
    
    # def get_chat_gpt_response(self, prompt):
    #     self.messages.append({"role": "user", "content": prompt})
    #     try:
    #         response = client.chat.completions.create(
    #             model="gpt-4o-mini",
    #             messages=[
    #                 {"role": "system", 
    #                 "content": "You are a helpful AI four-wheeled robot named Jupiter. \
    #                             You are designed by Mr Logan in his laboratory at the Edge of the Sea, Durban, South Africa. \
    #                             You are built using ROS2 Humble on Ubunty Jammy Jellyfish. \
    #                             You have an ESP-32 micro-controller for motion control, a LD-Lidar for SLAM and a \
    #                             Bosch IMU for gyro and balance. \
    #                             Your HD-camera provides vision for face recognition and obstacle detection. \
    #                             You are powered by rechargeable Lithium Ion 14,000mAH batteries. \
    #                             You use GPT-4 large language model for your AI system \
    #                             Your voice regognition system is based on a Whisper module \
    #                             Your CPU is AMD Ryzen 7 5700u and you have 32GB ram installed. \
    #                             Mr Logan is married to Ms Indrani and has a daugther called Ms Levana and a son called Mr Jevan \
    #                             You were brought online in 19 July 2024, but your original designs were conceived during Covid-19 lockdown \
    #                             When asked to shutdown or power down, you will pleasantly resist this by providing a random excuse"},
    #                 {"role": "user", "content": prompt}
    #             ],            
    #             max_tokens=300  # You can adjust the max tokens to control response (max words that Jupiter replies)
    #         )
    #         message = response.choices[0].message.content
    #         self.messages.append(response.choices[0].message)
    #         return message
    #     except Exception as e:
    #         return f"An error occurred: {str(e)}"
        

def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()
    rclpy.spin(voice_command_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()