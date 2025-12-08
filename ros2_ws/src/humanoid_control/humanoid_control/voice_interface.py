#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading

class VoiceInterface(Node):
    """Interface for processing voice commands"""

    def __init__(self):
        super().__init__('voice_interface')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Publisher for voice commands
        self.command_publisher = self.create_publisher(String, '/vla/command', 10)

        # Start voice recognition thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.get_logger().info("Voice interface initialized")

    def listen_continuously(self):
        """Continuously listen for voice commands"""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5)

                # Process audio
                command = self.recognizer.recognize_google(audio)
                self.get_logger().info(f"Heard command: {command}")

                # Publish command
                cmd_msg = String()
                cmd_msg.data = command
                self.command_publisher.publish(cmd_msg)

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().info("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Speech recognition error: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Voice interface error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    node = VoiceInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down voice interface")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()