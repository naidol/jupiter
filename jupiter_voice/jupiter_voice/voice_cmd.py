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
from jupiter_gpt.gpt_memory import GPTMemoryNode  # Import the GPTMemoryNode class from package
import re  # needed to remove punctuation from text strings


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")
        self.subscription = self.create_subscription(String, "/voice_cmd", self.callback, 10)
        self.user_id_sub = self.create_subscription(String, "/user_id", self.user_id_callback, 10)
        self.publisher = self.create_publisher(String, "/voice_tts", 10)
        self.new_user_id_pub = self.create_publisher(String,"/new_user_id", 10) # publish the new_user_id when unknown Human provides name
        self.messages = [] #this array is used to store conversations with GPT AI
        # Create an instance of GPTMemoryNode
        self.gpt_memory_node = GPTMemoryNode()
        self.user_id = 'Human'

    def user_id_callback(self, msg):
        self.user_id = msg.data


    # check for specific voice commands, else assume that its a standard voice request to GPT
    def callback(self, msg):       
        prompt = msg.data   # get the voice command message into text format
        prompt = re.sub(r'[^\w\s]','', prompt)
        prompt = prompt.strip()  # remove leading and trailing spaces
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
        elif 'my name is' in prompt.lower():
            new_user_id = prompt.lower().replace("my name is","")
            new_user_id = new_user_id.strip()
            new_user_id_msg = String()
            new_user_id_msg.data = new_user_id
            self.new_user_id_pub.publish(new_user_id_msg)
            self.gpt_memory_node.process_user_prompt(new_user_id, prompt)
        else:
            # Call GPTMemoryNode's method to process the user prompt
            self.gpt_memory_node.process_user_prompt(self.user_id, prompt)
             
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

def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()
    rclpy.spin(voice_command_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()