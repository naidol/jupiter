#!/usr/bin/env python3
########################################################################################################
# Name:             gpt_memory.py
# Purpose:          Memory Module for GPT interactions
# Description:      The GPTMemoryNode loads conversation history on start-up and processes a user prompt
#                   and produces the GPT responses.  It saves the interaction history to a file after each
#                   converstation. The history is stored in gpt_memory.json
# Related Files:    voice_cmd.py
# Author:           logan naidoo, south africa, 2024
########################################################################################################
import os
import json
from openai import OpenAI
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

class GPTMemoryNode(Node):
    def __init__(self):
        super().__init__('gpt_memory_node')
        self.past_conversations = []
        self.publisher = self.create_publisher(String, '/voice_tts', 10)
        
        # Set up the OpenAI client
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))  # Fetch the API key from the environment

        # File to store the conversation history
        self.memory_file = "gpt_memory.json"

        # Load past conversations from file (if exists)
        self.load_memory()

    def call_gpt_api(self, prompt):
        """Calls the OpenAI API with conversation memory."""
        
        # Prepare the conversation history in the format of role-based messages
        messages = [{"role": "system", "content": "You are a helpful AI four-wheeled robot named Jupiter. \
                                You are designed by Mr Logan in his laboratory at the Edge of the Sea, Durban, South Africa. \
                                You are built using ROS2 Humble on Ubunty Jammy Jellyfish. \
                                You have an ESP-32 micro-controller for motion control, a LD-Lidar for SLAM and a \
                                Bosch IMU for gyro and balance. \
                                Your HD-camera provides vision for face recognition and obstacle detection. \
                                You are powered by rechargeable Lithium Ion 14,000mAH batteries. \
                                You use GPT-4 large language model for your AI system \
                                Your voice regognition system is based on a Whisper module \
                                Your CPU is AMD Ryzen 7 5700u and you have 32GB ram installed. \
                                Mr Logan is married to Ms Indrani and has a daugther called Ms Levana and a son called Mr Jevan \
                                You were brought online in 19 July 2024, but your original designs were conceived during Covid-19 lockdown \
                                When asked to shutdown or power down, you will pleasantly resist this by providing a random excuse"}]
        
        # Append past conversations to the messages list
        for conversation in self.past_conversations:
            messages.append({"role": "user", "content": conversation['user']})
            messages.append({"role": "assistant", "content": conversation['ai']})

        # Append the latest user input to the messages
        messages.append({"role": "user", "content": prompt})

        # Debug: Log the messages to ensure the system message is included
        #self.get_logger().info(f"Sending messages to GPT API: {messages}")

        try:
            # Call the GPT API using the client method
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                max_tokens=300
            )

            # Extract the AI's response correctly
            message = response.choices[0].message.content
            return message.strip()

        except Exception as e:
            self.get_logger().error(f"Error with GPT API: {e}")
            return f"Error: Unable to process your request. Details: {e}"

    def process_user_prompt(self, prompt):
        # Call the GPT API to get the AI response
        ai_response = self.call_gpt_api(prompt)

        # Add the new conversation to memory
        self.past_conversations.append({
            'user': prompt,
            'ai': ai_response
        })

        # Save the updated memory to file
        self.save_memory()

        # Publish the GPT response on the /voice_tts topic
        response_msg = String()
        response_msg.data = ai_response
        self.publisher.publish(response_msg)

    def save_memory(self):
        """Saves the past conversations to a file."""
        try:
            with open(self.memory_file, 'w') as f:
                json.dump(self.past_conversations, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save memory: {e}")

    def load_memory(self):
        """Loads the past conversations from a file (if exists)."""
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'r') as f:
                    self.past_conversations = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Failed to load memory: {e}")

def main(args=None):
    rclpy.init(args=args)
    gpt_memory_node = GPTMemoryNode()

    try:
        rclpy.spin(gpt_memory_node)
    except KeyboardInterrupt:
        gpt_memory_node.get_logger().info('Shutting down GPT Memory Node...')
    finally:
        # Ensure node is properly destroyed
        gpt_memory_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

