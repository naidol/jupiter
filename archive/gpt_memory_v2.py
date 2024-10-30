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
        self.user_conversations = {}  # Dictionary to store conversation history for each user
        self.publisher = self.create_publisher(String, '/voice_tts', 10)
        
        # Set up the OpenAI client
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))  # Fetch the API key from the environment

        # Directory to store user conversation files
        self.home_dir = os.path.expanduser("~")
        self.memory_dir = os.path.join(self.home_dir,"jupiter_ws/src/jupiter/jupiter_gpt/gpt_memory_users")
        os.makedirs(self.memory_dir, exist_ok=True)  # Create the directory if it doesn't exist
        self.get_logger().info(self.memory_dir)

    def get_memory_file(self, user_id):
        """Returns the path to the memory file for a specific user."""
        return os.path.join(self.memory_dir, f"{user_id}_memory.json")

    def call_gpt_api(self, user_id, prompt):
        """Calls the OpenAI API with conversation memory for the given user."""
        
        # Always start with the system message to define the role of the AI
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
        
        # Append past conversations (user and assistant) for the current user
        if user_id in self.user_conversations:
            for conversation in self.user_conversations[user_id]:
                messages.append({"role": "user", "content": conversation['user']})
                messages.append({"role": "assistant", "content": conversation['ai']})

        # Append the latest user input
        messages.append({"role": "user", "content": prompt})

        try:
            # Call the GPT API
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                max_tokens=150
            )

            # Extract the AI's response
            message = response.choices[0].message.content
            return message.strip()

        except Exception as e:
            self.get_logger().error(f"Error with GPT API: {e}")
            return f"Error: Unable to process your request. Details: {e}"

    def process_user_prompt(self, user_id, prompt):
        """Processes a user prompt and generates a response for the specific user."""
        
        # Load user's conversation history if not already loaded
        if user_id not in self.user_conversations:
            self.load_memory(user_id)

        # Initialize user conversation history if not present
        if user_id not in self.user_conversations:
            # Set the initial conversation to recognize the user's name from the user_id
            self.user_conversations[user_id] = [
                {'user': "My name is " + user_id, 'ai': "Nice to meet you, " + user_id + "!"}
            ]
            # Save the memory immediately after initializing the conversation
            self.save_memory(user_id)

        # Call the GPT API to get the AI response for the specific user
        ai_response = self.call_gpt_api(user_id, prompt)

        # Add the new conversation to the user's memory
        self.user_conversations[user_id].append({
            'user': prompt,
            'ai': ai_response
        })

        # Save the updated memory for the user
        self.save_memory(user_id)

        # Publish the GPT response on the /voice_tts topic
        response_msg = String()
        response_msg.data = ai_response
        self.publisher.publish(response_msg)


    def save_memory(self, user_id):
        """Saves the conversation history for a specific user."""
        try:
            memory_file = self.get_memory_file(user_id)
            with open(memory_file, 'w') as f:
                json.dump(self.user_conversations[user_id], f)
        except Exception as e:
            self.get_logger().error(f"Failed to save memory for user {user_id}: {e}")

    def load_memory(self, user_id):
        """Loads the conversation history for a specific user from file."""
        memory_file = self.get_memory_file(user_id)
        if os.path.exists(memory_file):
            try:
                with open(memory_file, 'r') as f:
                    self.user_conversations[user_id] = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Failed to load memory for user {user_id}: {e}")
        else:
            self.user_conversations[user_id] = []  # Initialize with empty list if no history exists

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
    
