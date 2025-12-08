---
title: "Voice-to-Action: Mic → Whisper → LLM → ROS2"
description: "Implementing voice command processing pipeline from microphone to robot action"
learning_objectives:
  - "Understand the voice-to-action pipeline components"
  - "Implement speech recognition using Whisper"
  - "Integrate LLM for natural language understanding"
  - "Connect voice commands to ROS2 control systems"
---

# Voice-to-Action: Mic → Whisper → LLM → ROS2

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the voice-to-action pipeline components
- Implement speech recognition using Whisper
- Integrate LLM for natural language understanding
- Connect voice commands to ROS2 control systems

## Introduction

The voice-to-action pipeline represents a crucial component of natural human-robot interaction, enabling humanoid robots to understand and execute spoken commands. This pipeline transforms audio input from a microphone into meaningful robot actions through a series of processing steps: speech recognition, natural language understanding, and action generation. The combination of OpenAI's Whisper for speech-to-text, Large Language Models (LLMs) for language understanding, and ROS2 for robot control creates a powerful framework for intuitive robot interaction. This chapter will guide you through implementing this pipeline for humanoid robotics applications.

## Voice-to-Action Pipeline Architecture

### Overview of the Pipeline

The voice-to-action pipeline consists of four main stages:

1. **Audio Input**: Microphone capture and preprocessing
2. **Speech Recognition**: Converting speech to text using Whisper
3. **Language Understanding**: Interpreting text commands using LLMs
4. **Action Generation**: Converting understanding to ROS2 commands

### Pipeline Architecture Diagram

```
Microphone → Audio Preprocessing → Whisper → Text → LLM → Action Plan → ROS2 → Robot Action
```

### Component Integration

```python
# voice_to_action_pipeline.py - Complete voice-to-action pipeline
import asyncio
import queue
import threading
import time
import numpy as np
import sounddevice as sd
import whisper
import openai
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import json
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import logging

@dataclass
class VoiceCommand:
    """Data structure for voice commands"""
    text: str
    confidence: float
    timestamp: float
    speaker_id: Optional[str] = None

@dataclass
class ActionPlan:
    """Data structure for robot actions"""
    action_type: str
    parameters: Dict
    priority: int
    execution_time: Optional[float] = None

class AudioInputHandler:
    """
    Handle audio input from microphone with preprocessing
    """
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size

        # Audio buffer for continuous recording
        self.audio_buffer = queue.Queue()

        # Audio processing parameters
        self.silence_threshold = 0.01  # Threshold for detecting speech
        self.min_speech_duration = 0.5  # Minimum speech duration in seconds
        self.silence_duration = 1.0    # Duration of silence to trigger processing

        # Audio stream
        self.stream = None
        self.is_listening = False

        # Callback function for audio processing
        self.on_audio_ready = None

    def start_listening(self):
        """Start listening for audio input"""
        self.is_listening = True
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            blocksize=self.chunk_size,
            callback=self._audio_callback
        )
        self.stream.start()
        print("Started listening for audio input...")

    def stop_listening(self):
        """Stop listening for audio input"""
        self.is_listening = False
        if self.stream:
            self.stream.stop()
            self.stream.close()
        print("Stopped listening for audio input")

    def _audio_callback(self, indata, frames, time, status):
        """Callback function for audio stream"""
        if status:
            print(f"Audio stream status: {status}")

        # Add audio data to buffer
        audio_chunk = indata.copy()
        self.audio_buffer.put(audio_chunk)

        # Check for speech activity
        if self.on_audio_ready:
            # Simple energy-based voice activity detection
            audio_energy = np.mean(np.abs(audio_chunk))

            if audio_energy > self.silence_threshold:
                # Potential speech detected
                if not hasattr(self, '_speech_start_time'):
                    self._speech_start_time = time.current_time
            else:
                # Silence detected
                if hasattr(self, '_speech_start_time'):
                    speech_duration = time.current_time - self._speech_start_time
                    if speech_duration >= self.min_speech_duration:
                        # Trigger audio processing
                        self._process_audio_buffer()
                    delattr(self, '_speech_start_time')

    def _process_audio_buffer(self):
        """Process accumulated audio for speech recognition"""
        if self.on_audio_ready:
            # Collect audio data from buffer
            audio_data = []
            while not self.audio_buffer.empty():
                chunk = self.audio_buffer.get()
                audio_data.append(chunk)

            if audio_data:
                full_audio = np.concatenate(audio_data)
                self.on_audio_ready(full_audio)

    def get_audio_data(self):
        """Get accumulated audio data"""
        audio_data = []
        while not self.audio_buffer.empty():
            chunk = self.audio_buffer.get()
            audio_data.append(chunk)

        if audio_data:
            return np.concatenate(audio_data)
        return np.array([])

class WhisperProcessor:
    """
    Process audio using OpenAI's Whisper for speech recognition
    """
    def __init__(self, model_name="base"):
        # Load Whisper model
        self.model = whisper.load_model(model_name)
        self.sample_rate = 16000

    def transcribe_audio(self, audio_data, language="en"):
        """
        Transcribe audio data to text using Whisper

        Args:
            audio_data: Audio data as numpy array
            language: Target language for transcription

        Returns:
            transcription: Transcribed text with confidence
        """
        try:
            # Ensure audio is in correct format
            if len(audio_data.shape) > 1:
                # Take only first channel if stereo
                audio_data = audio_data[:, 0]

            # Pad or trim audio to minimum length (Whisper expects at least 0.1 seconds)
            min_samples = int(0.1 * self.sample_rate)
            if len(audio_data) < min_samples:
                padding = min_samples - len(audio_data)
                audio_data = np.pad(audio_data, (0, padding), mode='constant')

            # Transcribe audio
            result = self.model.transcribe(
                audio_data,
                language=language,
                temperature=0.0  # Deterministic output
            )

            # Calculate confidence based on log probabilities
            confidence = self._calculate_confidence(result)

            return VoiceCommand(
                text=result["text"].strip(),
                confidence=confidence,
                timestamp=time.time()
            )

        except Exception as e:
            print(f"Error in Whisper transcription: {e}")
            return VoiceCommand(
                text="",
                confidence=0.0,
                timestamp=time.time()
            )

    def _calculate_confidence(self, result):
        """Calculate confidence score from Whisper result"""
        # Whisper doesn't always provide token-level probabilities
        # Use a simple heuristic based on text length and timing
        if "segments" in result and result["segments"]:
            avg_logprob = np.mean([
                segment.get("avg_logprob", -1.0)
                for segment in result["segments"]
            ])
            # Convert log probability to confidence (0-1 range)
            confidence = max(0.0, min(1.0, (avg_logprob + 2.0) / 2.0))
            return confidence
        else:
            # Fallback confidence calculation
            return 0.5

class LLMCommandInterpreter:
    """
    Interpret voice commands using Large Language Models
    """
    def __init__(self, api_key: str, model_name: str = "gpt-3.5-turbo"):
        self.client = OpenAI(api_key=api_key)
        self.model_name = model_name

        # Define robot capabilities for context
        self.robot_capabilities = {
            "locomotion": ["walk", "move", "go", "step", "navigate"],
            "manipulation": ["grasp", "pick", "take", "hold", "place", "put"],
            "interaction": ["wave", "nod", "shake", "greet", "acknowledge"],
            "communication": ["speak", "say", "repeat", "tell", "announce"],
            "posture": ["sit", "stand", "lie", "balance", "posture"]
        }

        # Define action schema
        self.action_schema = {
            "type": "object",
            "properties": {
                "action_type": {
                    "type": "string",
                    "enum": ["navigation", "manipulation", "interaction", "communication", "posture"]
                },
                "parameters": {
                    "type": "object",
                    "properties": {
                        "target": {"type": "string"},
                        "location": {"type": "string"},
                        "direction": {"type": "string"},
                        "magnitude": {"type": "number"},
                        "duration": {"type": "number"}
                    }
                },
                "priority": {"type": "integer", "minimum": 0, "maximum": 10}
            },
            "required": ["action_type", "parameters"]
        }

    def interpret_command(self, voice_command: VoiceCommand) -> Optional[ActionPlan]:
        """
        Interpret voice command using LLM and return action plan

        Args:
            voice_command: Voice command with text and confidence

        Returns:
            action_plan: Structured action plan or None if invalid
        """
        if voice_command.confidence < 0.3:
            print(f"Command confidence too low ({voice_command.confidence}), ignoring")
            return None

        command_text = voice_command.text.lower().strip()
        if not command_text:
            return None

        # Construct prompt for LLM
        prompt = self._construct_interpretation_prompt(command_text)

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are a command interpreter for a humanoid robot. "
                            "Convert natural language commands into structured robot actions. "
                            "Respond only with valid JSON matching the provided schema."
                        )
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                response_format={"type": "json_object"},
                temperature=0.1  # Low temperature for consistency
            )

            # Parse response
            response_json = json.loads(response.choices[0].message.content)

            # Validate and create action plan
            action_plan = self._validate_and_create_action_plan(response_json)

            if action_plan:
                print(f"Interpreted command: '{command_text}' -> {action_plan.action_type}")
                return action_plan
            else:
                print(f"Failed to interpret command: {command_text}")
                return None

        except Exception as e:
            print(f"Error interpreting command with LLM: {e}")
            return None

    def _construct_interpretation_prompt(self, command_text: str) -> str:
        """Construct prompt for LLM interpretation"""
        prompt = f"""
        Interpret this robot command: "{command_text}"

        Available robot capabilities:
        - Locomotion: {', '.join(self.robot_capabilities['locomotion'])}
        - Manipulation: {', '.join(self.robot_capabilities['manipulation'])}
        - Interaction: {', '.join(self.robot_capabilities['interaction'])}
        - Communication: {', '.join(self.robot_capabilities['communication'])}
        - Posture: {', '.join(self.robot_capabilities['posture'])}

        Return a JSON object with:
        - action_type: one of ["navigation", "manipulation", "interaction", "communication", "posture"]
        - parameters: object with relevant parameters
        - priority: integer 0-10 (10 is highest priority)

        Example: "Please walk to the kitchen and pick up the red cup"
        {{
            "action_type": "navigation",
            "parameters": {{
                "target": "red cup",
                "location": "kitchen"
            }},
            "priority": 5
        }}
        """
        return prompt

    def _validate_and_create_action_plan(self, response_json: dict) -> Optional[ActionPlan]:
        """Validate LLM response and create action plan"""
        try:
            action_type = response_json.get("action_type")
            parameters = response_json.get("parameters", {})
            priority = response_json.get("priority", 5)

            # Validate action type
            valid_types = ["navigation", "manipulation", "interaction", "communication", "posture"]
            if action_type not in valid_types:
                return None

            # Validate priority
            if not isinstance(priority, int) or not (0 <= priority <= 10):
                priority = 5

            return ActionPlan(
                action_type=action_type,
                parameters=parameters,
                priority=priority
            )
        except Exception as e:
            print(f"Error validating action plan: {e}")
            return None

class ROS2ActionExecutor(Node):
    """
    Execute action plans through ROS2 control systems
    """
    def __init__(self):
        super().__init__('voice_to_action_executor')

        # Publishers for different action types
        self.nav_publisher = self.create_publisher(Twist, '/humanoid_robot/cmd_vel', 10)
        self.joint_publisher = self.create_publisher(JointState, '/humanoid_robot/joint_commands', 10)
        self.speech_publisher = self.create_publisher(String, '/tts_input', 10)
        self.action_status_publisher = self.create_publisher(String, '/voice_action_status', 10)

        # Subscribers for feedback
        self.feedback_subscriber = self.create_subscription(
            String, '/robot_feedback', self.feedback_callback, 10
        )

        # Action execution timer
        self.action_timer = self.create_timer(0.1, self.execute_pending_actions)

        # Pending actions queue
        self.action_queue = []
        self.current_action = None
        self.action_execution_start = None

        self.get_logger().info('Voice-to-Action executor initialized')

    def add_action(self, action_plan: ActionPlan):
        """Add action to execution queue"""
        self.action_queue.append(action_plan)
        self.get_logger().info(f'Added action to queue: {action_plan.action_type}')

    def execute_pending_actions(self):
        """Execute pending actions in the queue"""
        if not self.action_queue and not self.current_action:
            return

        if not self.current_action and self.action_queue:
            # Start next action
            self.current_action = self.action_queue.pop(0)
            self.action_execution_start = self.get_clock().now().nanoseconds / 1e9
            self.execute_action(self.current_action)

        elif self.current_action:
            # Check if current action is complete
            current_time = self.get_clock().now().nanoseconds / 1e9
            execution_time = current_time - self.action_execution_start

            # Simple timeout mechanism (in real implementation, use feedback)
            if execution_time > 5.0:  # 5 second timeout
                self.get_logger().info(f'Timeout for action: {self.current_action.action_type}')
                self.current_action = None

    def execute_action(self, action_plan: ActionPlan):
        """Execute a specific action plan"""
        self.get_logger().info(f'Executing action: {action_plan.action_type}')

        if action_plan.action_type == "navigation":
            self._execute_navigation(action_plan.parameters)
        elif action_plan.action_type == "manipulation":
            self._execute_manipulation(action_plan.parameters)
        elif action_plan.action_type == "interaction":
            self._execute_interaction(action_plan.parameters)
        elif action_plan.action_type == "communication":
            self._execute_communication(action_plan.parameters)
        elif action_plan.action_type == "posture":
            self._execute_posture(action_plan.parameters)

        # Publish action completion
        status_msg = String()
        status_msg.data = f"Completed: {action_plan.action_type}"
        self.action_status_publisher.publish(status_msg)

    def _execute_navigation(self, parameters):
        """Execute navigation action"""
        twist = Twist()

        # Parse navigation parameters
        direction = parameters.get("direction", "forward")
        magnitude = parameters.get("magnitude", 1.0)

        if direction == "forward":
            twist.linear.x = magnitude * 0.2  # Scale appropriately
        elif direction == "backward":
            twist.linear.x = -magnitude * 0.2
        elif direction == "left":
            twist.angular.z = magnitude * 0.5
        elif direction == "right":
            twist.angular.z = -magnitude * 0.5
        elif direction == "turn_left":
            twist.angular.z = magnitude * 0.5
        elif direction == "turn_right":
            twist.angular.z = -magnitude * 0.5

        self.nav_publisher.publish(twist)

    def _execute_manipulation(self, parameters):
        """Execute manipulation action"""
        # This would involve more complex joint control
        # For now, just publish a message indicating the action
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = ["left_shoulder", "left_elbow", "left_wrist",
                         "right_shoulder", "right_elbow", "right_wrist"]
        joint_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default positions

        # Modify based on manipulation parameters
        target = parameters.get("target", "")
        if "grasp" in target or "pick" in target:
            # Example: move arms to grasp position
            joint_cmd.position[2] = -0.5  # Left wrist flex
            joint_cmd.position[5] = -0.5  # Right wrist flex

        self.joint_publisher.publish(joint_cmd)

    def _execute_interaction(self, parameters):
        """Execute interaction action"""
        # For interaction actions like waving
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = ["left_shoulder", "left_elbow", "left_wrist"]
        joint_cmd.position = [0.5, 1.0, 0.0]  # Wave gesture

        self.joint_publisher.publish(joint_cmd)

    def _execute_communication(self, parameters):
        """Execute communication action"""
        text_to_say = parameters.get("text", "I heard a command")
        string_msg = String()
        string_msg.data = text_to_say
        self.speech_publisher.publish(string_msg)

    def _execute_posture(self, parameters):
        """Execute posture action"""
        # Change robot posture (sit, stand, etc.)
        posture = parameters.get("posture", "stand")
        joint_cmd = JointState()

        if posture == "sit":
            # Simplified sitting position
            joint_cmd.name = ["left_hip", "left_knee", "left_ankle",
                             "right_hip", "right_knee", "right_ankle"]
            joint_cmd.position = [-0.5, 1.0, -0.5, -0.5, 1.0, -0.5]
        elif posture == "stand":
            # Simplified standing position
            joint_cmd.name = ["left_hip", "left_knee", "left_ankle",
                             "right_hip", "right_knee", "right_ankle"]
            joint_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        self.joint_publisher.publish(joint_cmd)

    def feedback_callback(self, msg):
        """Handle feedback from robot"""
        self.get_logger().info(f'Received feedback: {msg.data}')

class VoiceToActionSystem:
    """
    Complete voice-to-action system integrating all components
    """
    def __init__(self, openai_api_key: str):
        # Initialize components
        self.audio_handler = AudioInputHandler()
        self.whisper_processor = WhisperProcessor(model_name="base")
        self.llm_interpreter = LLMCommandInterpreter(openai_api_key)

        # Initialize ROS2
        rclpy.init()
        self.action_executor = ROS2ActionExecutor()

        # Connect audio handler to processing chain
        self.audio_handler.on_audio_ready = self._process_audio

        # Internal state
        self.pending_audio = None
        self.processing_lock = threading.Lock()

        # Statistics
        self.command_count = 0
        self.error_count = 0

        print("Voice-to-Action system initialized")

    def _process_audio(self, audio_data):
        """Process audio through the complete pipeline"""
        with self.processing_lock:
            try:
                # Step 1: Speech recognition with Whisper
                voice_command = self.whisper_processor.transcribe_audio(audio_data)

                if voice_command.confidence > 0.3 and voice_command.text.strip():
                    print(f"Recognized: '{voice_command.text}' (confidence: {voice_command.confidence:.2f})")

                    # Step 2: Language understanding with LLM
                    action_plan = self.llm_interpreter.interpret_command(voice_command)

                    if action_plan:
                        print(f"Generated action: {action_plan.action_type}")

                        # Step 3: Execute action through ROS2
                        self.action_executor.add_action(action_plan)

                        self.command_count += 1
                    else:
                        print("Could not interpret command")
                        self.error_count += 1
                else:
                    print(f"Audio confidence too low or empty: {voice_command.confidence:.2f}")
                    self.error_count += 1

            except Exception as e:
                print(f"Error in voice processing pipeline: {e}")
                self.error_count += 1

    def start_listening(self):
        """Start the voice-to-action system"""
        print("Starting voice-to-action system...")

        # Start audio input
        self.audio_handler.start_listening()

        # Start ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=self._spin_ros)
        ros_thread.daemon = True
        ros_thread.start()

        print("Voice-to-action system is running. Press Ctrl+C to stop.")

        try:
            # Keep main thread alive
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping voice-to-action system...")
            self.stop()

    def _spin_ros(self):
        """Spin ROS2 node"""
        rclpy.spin(self.action_executor)

    def stop(self):
        """Stop the voice-to-action system"""
        print("Stopping voice-to-action system...")

        # Stop audio input
        self.audio_handler.stop_listening()

        # Shutdown ROS2
        self.action_executor.destroy_node()
        rclpy.shutdown()

        print(f"System stopped. Processed {self.command_count} commands, {self.error_count} errors.")

def main():
    """Main function to run the voice-to-action system"""
    import os

    # Get OpenAI API key from environment
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set")
        return

    # Initialize and start the system
    vta_system = VoiceToActionSystem(api_key)

    try:
        vta_system.start_listening()
    except Exception as e:
        print(f"Error running voice-to-action system: {e}")
    finally:
        vta_system.stop()

if __name__ == "__main__":
    main()
```

## Advanced Whisper Integration

### Whisper Model Optimization

```python
# whisper_optimization.py - Optimized Whisper integration
import whisper
import torch
import numpy as np
from transformers import pipeline
import asyncio
from concurrent.futures import ThreadPoolExecutor
import time

class OptimizedWhisperProcessor:
    """
    Optimized Whisper processor with performance enhancements
    """
    def __init__(self, model_name="base", use_gpu=True):
        # Check GPU availability
        self.device = "cuda" if torch.cuda.is_available() and use_gpu else "cpu"

        # Load model with optimizations
        self.model = whisper.load_model(model_name).to(self.device)

        # Use fp16 for faster inference on GPU
        if self.device == "cuda":
            self.model = self.model.half()

        # Thread pool for parallel processing
        self.executor = ThreadPoolExecutor(max_workers=2)

        # Audio processing parameters
        self.sample_rate = 16000
        self.max_audio_length = 30  # Maximum audio length in seconds
        self.min_audio_length = 0.5  # Minimum audio length in seconds

        # Caching for repeated phrases
        self.transcription_cache = {}
        self.cache_size_limit = 100

    def transcribe_audio_async(self, audio_data, language="en"):
        """
        Asynchronously transcribe audio data
        """
        future = self.executor.submit(
            self._transcribe_with_cache,
            audio_data,
            language
        )
        return future

    def _transcribe_with_cache(self, audio_data, language):
        """
        Transcribe audio with caching
        """
        # Create cache key from audio hash and language
        audio_hash = hash(audio_data.tobytes()) % 1000000
        cache_key = f"{audio_hash}_{language}"

        # Check cache first
        if cache_key in self.transcription_cache:
            cached_result, timestamp = self.transcription_cache[cache_key]
            # Check if cache is still valid (less than 5 minutes old)
            if time.time() - timestamp < 300:
                return cached_result

        # Process audio if not in cache
        result = self._transcribe_audio_internal(audio_data, language)

        # Cache result
        self._add_to_cache(cache_key, result)

        return result

    def _transcribe_audio_internal(self, audio_data, language):
        """
        Internal transcription method
        """
        try:
            # Validate audio data
            if len(audio_data) == 0:
                return VoiceCommand("", 0.0, time.time())

            # Ensure proper format
            if len(audio_data.shape) > 1:
                audio_data = audio_data[:, 0]

            # Calculate duration
            duration = len(audio_data) / self.sample_rate

            # Validate duration
            if duration < self.min_audio_length:
                return VoiceCommand("", 0.0, time.time())
            elif duration > self.max_audio_length:
                # Trim audio to maximum length
                max_samples = int(self.max_audio_length * self.sample_rate)
                audio_data = audio_data[:max_samples]

            # Pad if necessary
            min_samples = int(self.min_audio_length * self.sample_rate)
            if len(audio_data) < min_samples:
                padding = min_samples - len(audio_data)
                audio_data = np.pad(audio_data, (0, padding), mode='constant')

            # Transcribe
            with torch.no_grad():
                result = self.model.transcribe(
                    audio_data,
                    language=language,
                    temperature=0.0,
                    compression_ratio_threshold=2.4,
                    logprob_threshold=-1.0,
                    no_speech_threshold=0.6
                )

            # Calculate confidence
            confidence = self._calculate_advanced_confidence(result)

            return VoiceCommand(
                text=result["text"].strip(),
                confidence=confidence,
                timestamp=time.time()
            )

        except Exception as e:
            print(f"Error in Whisper transcription: {e}")
            return VoiceCommand("", 0.0, time.time())

    def _calculate_advanced_confidence(self, result):
        """
        Calculate confidence using multiple metrics
        """
        if "segments" not in result or not result["segments"]:
            return 0.5

        # Collect metrics
        avg_logprobs = []
        no_speech_probs = []
        compression_ratios = []

        for segment in result["segments"]:
            if "avg_logprob" in segment:
                avg_logprobs.append(segment["avg_logprob"])
            if "no_speech_prob" in segment:
                no_speech_probs.append(segment["no_speech_prob"])

        # Calculate composite confidence
        confidence = 0.0

        if avg_logprobs:
            avg_logprob = np.mean(avg_logprobs)
            # Convert logprob to confidence (higher logprob = higher confidence)
            logprob_conf = max(0.0, min(1.0, (avg_logprob + 1.0) / 2.0))
            confidence += 0.4 * logprob_conf

        if no_speech_probs:
            avg_no_speech = np.mean(no_speech_probs)
            # Lower no_speech_prob = more confidence in speech
            no_speech_conf = max(0.0, min(1.0, 1.0 - avg_no_speech))
            confidence += 0.3 * no_speech_conf

        # Length-based confidence (very short transcriptions might be unreliable)
        text_length = len(result.get("text", ""))
        length_conf = min(1.0, text_length / 50.0)  # Good confidence for 50+ character transcriptions
        confidence += 0.3 * length_conf

        return min(1.0, confidence)

    def _add_to_cache(self, cache_key, result):
        """
        Add result to cache with size management
        """
        if len(self.transcription_cache) >= self.cache_size_limit:
            # Remove oldest entries
            oldest_key = min(self.transcription_cache.keys(),
                           key=lambda k: self.transcription_cache[k][1])
            del self.transcription_cache[oldest_key]

        self.transcription_cache[cache_key] = (result, time.time())

    def batch_transcribe(self, audio_batch, language="en"):
        """
        Transcribe multiple audio segments efficiently
        """
        futures = []
        for audio_data in audio_batch:
            future = self.transcribe_audio_async(audio_data, language)
            futures.append(future)

        results = []
        for future in futures:
            result = future.result()  # Wait for completion
            results.append(result)

        return results

    def get_model_info(self):
        """
        Get information about the loaded model
        """
        return {
            "model_name": self.model.name,
            "device": self.device,
            "is_quantized": hasattr(self.model, 'quantize'),
            "cache_size": len(self.transcription_cache),
            "cache_limit": self.cache_size_limit
        }
```

## LLM Integration for Command Understanding

### Advanced LLM Command Processing

```python
# llm_command_processing.py - Advanced LLM integration
import openai
from openai import OpenAI
import json
import re
from typing import Dict, List, Optional, Tuple, Any
import asyncio
import time
from dataclasses import dataclass, asdict
import logging

@dataclass
class ParsedCommand:
    """Structured representation of a parsed command"""
    intent: str
    entities: Dict[str, Any]
    confidence: float
    original_text: str
    timestamp: float

@dataclass
class RobotCapability:
    """Definition of robot capabilities"""
    name: str
    description: str
    parameters: List[str]
    examples: List[str]

class AdvancedLLMInterpreter:
    """
    Advanced LLM-based command interpreter with context awareness
    """
    def __init__(self, api_key: str, model_name: str = "gpt-4-turbo",
                 temperature: float = 0.1):
        self.client = OpenAI(api_key=api_key)
        self.model_name = model_name
        self.temperature = temperature

        # Robot capabilities definition
        self.robot_capabilities = [
            RobotCapability(
                name="navigation",
                description="Move the robot to different locations",
                parameters=["target_location", "distance", "direction"],
                examples=["Go to the kitchen", "Move forward 2 meters", "Turn left"]
            ),
            RobotCapability(
                name="manipulation",
                description="Manipulate objects with robot arms/hands",
                parameters=["target_object", "action", "location"],
                examples=["Pick up the red cup", "Place the book on the table", "Grasp the handle"]
            ),
            RobotCapability(
                name="interaction",
                description="Social interaction behaviors",
                parameters=["target_person", "behavior_type", "context"],
                examples=["Wave to John", "Nod your head", "Shake hands with Sarah"]
            ),
            RobotCapability(
                name="communication",
                description="Speak or communicate messages",
                parameters=["message", "recipient", "tone"],
                examples=["Say hello to everyone", "Tell me a joke", "Announce dinner time"]
            ),
            RobotCapability(
                name="posture",
                description="Change body posture or stance",
                parameters=["posture_type", "duration"],
                examples=["Sit down", "Stand up", "Balance yourself"]
            )
        ]

        # Context management
        self.context_history = []
        self.max_context_length = 10

        # Command cache for frequently used commands
        self.command_cache = {}
        self.cache_ttl = 300  # 5 minutes

    async def interpret_command_async(self, voice_command: VoiceCommand) -> Optional[ParsedCommand]:
        """
        Asynchronously interpret voice command using LLM
        """
        if voice_command.confidence < 0.3:
            return None

        # Check cache first
        cache_key = voice_command.text.lower().strip()
        if cache_key in self.command_cache:
            cached_result, timestamp = self.command_cache[cache_key]
            if time.time() - timestamp < self.cache_ttl:
                return cached_result

        # Prepare context
        context = self._build_context(voice_command.text)

        # Create structured prompt
        prompt = self._create_structured_prompt(voice_command.text, context)

        try:
            # Call LLM with structured output
            response = await self._call_llm_with_retry(prompt)

            if response:
                parsed_command = self._parse_llm_response(response)

                # Cache the result
                self.command_cache[cache_key] = (parsed_command, time.time())

                # Update context history
                self._update_context_history(voice_command, parsed_command)

                return parsed_command

        except Exception as e:
            logging.error(f"Error in LLM command interpretation: {e}")

        return None

    def _build_context(self, current_command: str) -> Dict[str, Any]:
        """
        Build contextual information for command interpretation
        """
        return {
            "capabilities": [asdict(cap) for cap in self.robot_capabilities],
            "recent_commands": self.context_history[-3:],  # Last 3 commands
            "current_command": current_command,
            "timestamp": time.time()
        }

    def _create_structured_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create a structured prompt for LLM interpretation
        """
        capabilities_str = "\n".join([
            f"- {cap['name']}: {cap['description']} "
            f"Parameters: {', '.join(cap['parameters']) if cap['parameters'] else 'none'}"
            for cap in context['capabilities']
        ])

        recent_commands_str = "\n".join([
            f"Previous: {cmd['original_text']} -> {cmd['intent']}"
            for cmd in context['recent_commands']
        ]) if context['recent_commands'] else "No recent commands"

        prompt = f"""
        You are an intelligent command interpreter for a humanoid robot.
        Interpret the following command: "{command}"

        Robot Capabilities:
        {capabilities_str}

        Recent Interaction Context:
        {recent_commands_str}

        Available intents: navigation, manipulation, interaction, communication, posture

        Respond with a JSON object containing:
        {{
            "intent": "the identified intent",
            "entities": {{"parameter_name": "extracted_value", ...}},
            "confidence": confidence_score_between_0_and_1
        }}

        Examples:
        Input: "Please walk to the kitchen counter"
        Output: {{"intent": "navigation", "entities": {{"target_location": "kitchen counter"}}, "confidence": 0.9}}

        Input: "Pick up the red cup on the table"
        Output: {{"intent": "manipulation", "entities": {{"target_object": "red cup", "location": "table"}}, "confidence": 0.85}}

        Input: "Say hello to everyone in the room"
        Output: {{"intent": "communication", "entities": {{"message": "hello", "recipient": "everyone"}}, "confidence": 0.92}}
        """

        return prompt

    async def _call_llm_with_retry(self, prompt: str, max_retries: int = 3) -> Optional[Dict]:
        """
        Call LLM with retry logic
        """
        for attempt in range(max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.model_name,
                    messages=[
                        {
                            "role": "system",
                            "content": (
                                "You are a precise command interpreter. "
                                "Always respond with valid JSON matching the schema."
                            )
                        },
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ],
                    response_format={"type": "json_object"},
                    temperature=self.temperature,
                    max_tokens=200
                )

                # Parse the response
                response_text = response.choices[0].message.content
                parsed_response = json.loads(response_text)

                return parsed_response

            except json.JSONDecodeError:
                logging.warning(f"Attempt {attempt + 1}: Invalid JSON response")
                if attempt == max_retries - 1:
                    return None
            except Exception as e:
                logging.warning(f"Attempt {attempt + 1}: LLM call failed: {e}")
                if attempt == max_retries - 1:
                    return None

                # Wait before retry
                await asyncio.sleep(0.5 * (attempt + 1))

        return None

    def _parse_llm_response(self, response: Dict) -> Optional[ParsedCommand]:
        """
        Parse LLM response into structured command
        """
        try:
            intent = response.get("intent", "").lower()
            entities = response.get("entities", {})
            confidence = response.get("confidence", 0.5)

            # Validate intent
            valid_intents = ["navigation", "manipulation", "interaction", "communication", "posture"]
            if intent not in valid_intents:
                logging.warning(f"Invalid intent: {intent}")
                return None

            # Validate confidence
            if not isinstance(confidence, (int, float)) or not (0 <= confidence <= 1):
                confidence = 0.5

            return ParsedCommand(
                intent=intent,
                entities=entities,
                confidence=confidence,
                original_text=response.get("original_text", ""),
                timestamp=time.time()
            )

        except Exception as e:
            logging.error(f"Error parsing LLM response: {e}")
            return None

    def _update_context_history(self, voice_command: VoiceCommand, parsed_command: ParsedCommand):
        """
        Update command context history
        """
        context_entry = {
            "original_text": voice_command.text,
            "intent": parsed_command.intent,
            "entities": parsed_command.entities,
            "confidence": parsed_command.confidence,
            "timestamp": parsed_command.timestamp
        }

        self.context_history.append(context_entry)

        # Keep only recent entries
        if len(self.context_history) > self.max_context_length:
            self.context_history = self.context_history[-self.max_context_length:]

    def get_context_summary(self) -> Dict[str, Any]:
        """
        Get summary of current context
        """
        return {
            "total_commands_processed": len(self.context_history),
            "recent_intents": [entry["intent"] for entry in self.context_history[-5:]],
            "cache_size": len(self.command_cache),
            "capabilities_count": len(self.robot_capabilities)
        }

    def clear_context(self):
        """
        Clear the context history
        """
        self.context_history.clear()
        logging.info("Context history cleared")

class ContextAwareInterpreter:
    """
    Interpreter with advanced context awareness
    """
    def __init__(self, base_interpreter: AdvancedLLMInterpreter):
        self.base_interpreter = base_interpreter
        self.entity_resolution_cache = {}
        self.max_resolution_cache_size = 50

    async def interpret_with_context(self, voice_command: VoiceCommand,
                                   environment_context: Dict = None) -> Optional[ParsedCommand]:
        """
        Interpret command with environmental context
        """
        # Enhance command with environmental context
        enhanced_command = await self._enhance_with_environment(
            voice_command, environment_context
        )

        # Use base interpreter
        parsed_command = await self.base_interpreter.interpret_command_async(
            VoiceCommand(
                text=enhanced_command,
                confidence=voice_command.confidence,
                timestamp=voice_command.timestamp
            )
        )

        return parsed_command

    async def _enhance_with_environment(self, voice_command: VoiceCommand,
                                      env_context: Dict = None) -> str:
        """
        Enhance command with environmental context
        """
        if not env_context:
            return voice_command.text

        # Example: resolve pronouns and ambiguous references
        command_text = voice_command.text.lower()

        # Enhance with location information
        if 'current_location' in env_context:
            command_text = command_text.replace('there', env_context['current_location'])
            command_text = command_text.replace('here', env_context['current_location'])

        # Enhance with visible objects
        if 'visible_objects' in env_context:
            objects = env_context['visible_objects']
            # Resolve ambiguous object references
            for obj in objects:
                # If command mentions "it" and there's a recently seen object
                if 'it' in command_text and len(objects) == 1:
                    command_text = command_text.replace('it', obj)

        return command_text

    def resolve_entities(self, entities: Dict[str, str],
                        env_context: Dict = None) -> Dict[str, str]:
        """
        Resolve ambiguous entities using environmental context
        """
        if not env_context:
            return entities

        resolved_entities = entities.copy()

        # Resolve object references
        if 'target_object' in entities and 'objects_in_scene' in env_context:
            target_obj = entities['target_object']
            scene_objects = env_context['objects_in_scene']

            # Find best match for ambiguous object references
            if target_obj in ['it', 'that', 'this']:
                # Use most recently detected object or most prominent object
                if scene_objects:
                    resolved_entities['target_object'] = scene_objects[0]

        # Resolve location references
        if 'target_location' in entities and 'known_locations' in env_context:
            target_loc = entities['target_location']
            known_locs = env_context['known_locations']

            # Resolve relative locations
            if target_loc in ['kitchen', 'living room', 'bedroom']:
                # Find the specific instance of the location
                for loc in known_locs:
                    if target_loc in loc.lower():
                        resolved_entities['target_location'] = loc
                        break

        return resolved_entities
```

## ROS2 Integration and Action Execution

### Advanced ROS2 Action Execution

```python
# ros2_action_execution.py - Advanced ROS2 action execution
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float64
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import navigation and manipulation actions
try:
    from nav2_msgs.action import NavigateToPose
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:
    print("Navigation and control actions not available, using mock implementations")

class AdvancedROS2ActionExecutor(Node):
    """
    Advanced ROS2 action executor with state management and feedback
    """
    def __init__(self):
        super().__init__('advanced_voice_action_executor')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.joint_traj_client = ActionClient(self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')

        # Publishers for different action types
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.speech_pub = self.create_publisher(String, '/tts_input', 10)
        self.status_pub = self.create_publisher(String, '/action_status', 10)

        # State management
        self.current_action = None
        self.action_queue = []
        self.is_executing = False

        # Action timeouts
        self.action_timeouts = {
            'navigation': 60.0,  # 60 seconds for navigation
            'manipulation': 30.0,  # 30 seconds for manipulation
            'interaction': 10.0,   # 10 seconds for interaction
            'communication': 5.0,  # 5 seconds for communication
            'posture': 15.0        # 15 seconds for posture
        }

        # Timer for action execution
        self.action_timer = self.create_timer(0.1, self.execute_next_action)

        # Callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info('Advanced ROS2 Action Executor initialized')

    def queue_action(self, parsed_command):
        """
        Queue action for execution
        """
        self.action_queue.append(parsed_command)
        self.get_logger().info(f'Queued action: {parsed_command.intent}')

    def execute_next_action(self):
        """
        Execute the next action in the queue
        """
        if self.is_executing or not self.action_queue:
            return

        # Get next action
        action = self.action_queue.pop(0)
        self.current_action = action
        self.is_executing = True

        # Execute based on action type
        if action.intent == 'navigation':
            self._execute_navigation_action(action)
        elif action.intent == 'manipulation':
            self._execute_manipulation_action(action)
        elif action.intent == 'interaction':
            self._execute_interaction_action(action)
        elif action.intent == 'communication':
            self._execute_communication_action(action)
        elif action.intent == 'posture':
            self._execute_posture_action(action)
        else:
            self.get_logger().error(f'Unknown action type: {action.intent}')
            self._finish_action()

    def _execute_navigation_action(self, action):
        """
        Execute navigation action
        """
        try:
            # Wait for navigation action server
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
                self._finish_action()
                return

            # Create navigation goal
            goal_msg = NavigateToPose.Goal()

            # Parse navigation parameters
            entities = action.entities
            if 'target_location' in entities:
                # In a real system, you'd have a map of locations
                # For now, we'll use a simple coordinate system
                location = entities['target_location']

                # Mock location mapping
                location_coords = {
                    'kitchen': (2.0, 0.0, 0.0),
                    'living room': (0.0, 2.0, 0.0),
                    'bedroom': (-2.0, 0.0, 0.0),
                    'office': (0.0, -2.0, 0.0)
                }

                if location in location_coords:
                    x, y, theta = location_coords[location]
                    goal_msg.pose.pose.position.x = x
                    goal_msg.pose.pose.position.y = y
                    goal_msg.pose.pose.orientation.z = theta
                    goal_msg.pose.header.frame_id = 'map'

            # Send navigation goal
            self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self._nav_feedback_callback
            ).add_done_callback(self._nav_goal_response_callback)

        except Exception as e:
            self.get_logger().error(f'Error executing navigation: {e}')
            self._finish_action()

    def _execute_manipulation_action(self, action):
        """
        Execute manipulation action
        """
        try:
            # Wait for joint trajectory action server
            if not self.joint_traj_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Joint trajectory action server not available')
                self._finish_action()
                return

            # Create joint trajectory goal
            goal_msg = FollowJointTrajectory.Goal()

            # Define joint trajectory for manipulation
            trajectory = JointTrajectory()
            trajectory.joint_names = [
                'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
                'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow'
            ]

            # Create trajectory points based on action entities
            point = JointTrajectoryPoint()
            if 'target_object' in action.entities:
                obj = action.entities['target_object']
                if 'cup' in obj or 'glass' in obj:
                    # Reach for cup
                    point.positions = [0.5, 0.3, -0.8, 0.5, -0.3, -0.8]  # Left and right arm positions
                elif 'book' in obj:
                    # Reach for book (different position)
                    point.positions = [0.2, 0.5, -0.3, 0.2, -0.5, -0.3]
                else:
                    # Default reach position
                    point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            point.time_from_start.sec = 2  # 2 seconds to reach
            trajectory.points.append(point)

            goal_msg.trajectory = trajectory

            # Send manipulation goal
            self.joint_traj_client.send_goal_async(goal_msg).add_done_callback(
                self._manipulation_goal_response_callback
            )

        except Exception as e:
            self.get_logger().error(f'Error executing manipulation: {e}')
            self._finish_action()

    def _execute_interaction_action(self, action):
        """
        Execute interaction action (e.g., waving)
        """
        try:
            # Publish joint commands for interaction
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow']

            # Wave gesture
            if 'wave' in action.original_text.lower():
                joint_cmd.position = [0.5, 0.0, 1.0]  # Raise and bend left arm
            elif 'nod' in action.original_text.lower():
                joint_cmd.position = [0.0, 0.0, 0.0]  # Default position with head nod simulation
                # In a real system, you'd publish to a head controller

            self.joint_cmd_pub.publish(joint_cmd)

            # Schedule return to neutral position
            self.create_timer(2.0, self._return_to_neutral)

        except Exception as e:
            self.get_logger().error(f'Error executing interaction: {e}')

        finally:
            self._finish_action()

    def _execute_communication_action(self, action):
        """
        Execute communication action (speech)
        """
        try:
            # Get message from entities or use original command
            message = action.entities.get('message', action.original_text)

            # Publish to TTS system
            speech_msg = String()
            speech_msg.data = message
            self.speech_pub.publish(speech_msg)

        except Exception as e:
            self.get_logger().error(f'Error executing communication: {e}')

        finally:
            self._finish_action()

    def _execute_posture_action(self, action):
        """
        Execute posture change action
        """
        try:
            posture = action.entities.get('posture_type', 'stand')
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()

            if posture == 'sit':
                # Simplified sitting posture
                joint_cmd.name = ['left_hip', 'left_knee', 'left_ankle',
                                 'right_hip', 'right_knee', 'right_ankle',
                                 'torso_pitch']
                joint_cmd.position = [-0.5, 1.0, -0.5, -0.5, 1.0, -0.5, 0.3]
            elif posture == 'stand':
                # Standing posture
                joint_cmd.name = ['left_hip', 'left_knee', 'left_ankle',
                                 'right_hip', 'right_knee', 'right_ankle',
                                 'torso_pitch']
                joint_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif posture == 'crouch':
                # Crouching posture
                joint_cmd.name = ['left_hip', 'left_knee', 'left_ankle',
                                 'right_hip', 'right_knee', 'right_ankle',
                                 'torso_pitch']
                joint_cmd.position = [-0.3, 0.6, -0.3, -0.3, 0.6, -0.3, 0.1]

            self.joint_cmd_pub.publish(joint_cmd)

        except Exception as e:
            self.get_logger().error(f'Error executing posture: {e}')

        finally:
            self._finish_action()

    def _return_to_neutral(self):
        """
        Return joints to neutral position after interaction
        """
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow']
        joint_cmd.position = [0.0, 0.0, 0.0]  # Neutral position
        self.joint_cmd_pub.publish(joint_cmd)

    def _nav_feedback_callback(self, feedback_msg):
        """
        Navigation feedback callback
        """
        self.get_logger().debug(f'Navigation feedback: {feedback_msg}')

    def _nav_goal_response_callback(self, future):
        """
        Navigation goal response callback
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self._finish_action()
            return

        self.get_logger().info('Navigation goal accepted')
        goal_handle.get_result_async().add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """
        Navigation result callback
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

        self._finish_action()

    def _manipulation_goal_response_callback(self, future):
        """
        Manipulation goal response callback
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Manipulation goal rejected')
            self._finish_action()
            return

        self.get_logger().info('Manipulation goal accepted')
        goal_handle.get_result_async().add_done_callback(self._manipulation_result_callback)

    def _manipulation_result_callback(self, future):
        """
        Manipulation result callback
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Manipulation succeeded')
        else:
            self.get_logger().info(f'Manipulation failed with status: {status}')

        self._finish_action()

    def _finish_action(self):
        """
        Finish current action and prepare for next
        """
        self.is_executing = False
        self.current_action = None

        # Publish status
        status_msg = String()
        status_msg.data = "Action completed"
        self.status_pub.publish(status_msg)

    def get_execution_status(self):
        """
        Get current execution status
        """
        return {
            'is_executing': self.is_executing,
            'current_action': self.current_action.intent if self.current_action else None,
            'queue_size': len(self.action_queue),
            'action_timeouts': self.action_timeouts
        }
```

## Practical Implementation and Testing

### Complete Voice-to-Action System

```python
# complete_voice_system.py - Complete integrated voice-to-action system
import asyncio
import threading
import signal
import sys
from typing import Dict, Optional
import json

class CompleteVoiceToActionSystem:
    """
    Complete integrated voice-to-action system
    """
    def __init__(self, openai_api_key: str):
        # Initialize components
        self.whisper_processor = OptimizedWhisperProcessor(model_name="base")
        self.llm_interpreter = AdvancedLLMInterpreter(openai_api_key)
        self.context_interpreter = ContextAwareInterpreter(self.llm_interpreter)

        # Initialize ROS2
        rclpy.init()
        self.ros_executor = AdvancedROS2ActionExecutor()

        # Audio handler
        self.audio_handler = AudioInputHandler()
        self.audio_handler.on_audio_ready = self._process_audio

        # System state
        self.is_running = False
        self.stats = {
            'commands_processed': 0,
            'errors': 0,
            'average_confidence': 0.0
        }

        # Environmental context (would be updated from sensors)
        self.environment_context = {
            'current_location': 'unknown',
            'visible_objects': [],
            'known_locations': ['kitchen', 'living room', 'bedroom', 'office'],
            'objects_in_scene': []
        }

    def _process_audio(self, audio_data):
        """
        Process audio through complete pipeline
        """
        try:
            # Step 1: Whisper transcription
            voice_command = self.whisper_processor._transcribe_audio_internal(
                audio_data, "en"
            )

            if voice_command.confidence < 0.3 or not voice_command.text.strip():
                return

            print(f"Recognized: '{voice_command.text}' (confidence: {voice_command.confidence:.2f})")

            # Step 2: LLM interpretation with context
            parsed_command = asyncio.run(
                self.context_interpreter.interpret_with_context(
                    voice_command, self.environment_context
                )
            )

            if parsed_command:
                print(f"Parsed command: {parsed_command.intent} with confidence {parsed_command.confidence:.2f}")

                # Step 3: Execute through ROS2
                self.ros_executor.queue_action(parsed_command)

                # Update statistics
                self.stats['commands_processed'] += 1
                self.stats['average_confidence'] = (
                    (self.stats['average_confidence'] * (self.stats['commands_processed'] - 1) +
                     parsed_command.confidence) / self.stats['commands_processed']
                )
            else:
                self.stats['errors'] += 1
                print("Could not parse command")

        except Exception as e:
            self.stats['errors'] += 1
            print(f"Error in audio processing: {e}")

    def start(self):
        """
        Start the complete voice-to-action system
        """
        print("Starting Complete Voice-to-Action System...")

        # Start audio input
        self.audio_handler.start_listening()
        self.is_running = True

        # Start ROS2 in separate thread
        ros_thread = threading.Thread(target=self._run_ros, daemon=True)
        ros_thread.start()

        print("System is running. Say commands to control the robot.")
        print("Press Ctrl+C to stop.")

        # Signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

        try:
            # Keep main thread alive
            while self.is_running:
                # Print stats periodically
                if self.stats['commands_processed'] % 10 == 0:
                    self._print_stats()
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _run_ros(self):
        """
        Run ROS2 spin in separate thread
        """
        rclpy.spin(self.ros_executor)

    def _signal_handler(self, signum, frame):
        """
        Handle shutdown signal
        """
        print("\nShutdown signal received...")
        self.stop()

    def stop(self):
        """
        Stop the system gracefully
        """
        print("Stopping voice-to-action system...")

        self.is_running = False

        # Stop audio
        self.audio_handler.stop_listening()

        # Shutdown ROS2
        self.ros_executor.destroy_node()
        rclpy.shutdown()

        # Print final statistics
        self._print_stats(final=True)

    def _print_stats(self, final=False):
        """
        Print system statistics
        """
        total_ops = self.stats['commands_processed'] + self.stats['errors']
        success_rate = (self.stats['commands_processed'] / total_ops * 100) if total_ops > 0 else 0

        stats_str = (
            f"Stats - Commands: {self.stats['commands_processed']}, "
            f"Errors: {self.stats['errors']}, "
            f"Success Rate: {success_rate:.1f}%, "
            f"Avg Confidence: {self.stats['average_confidence']:.2f}"
        )

        if final:
            print(f"\nFinal {stats_str}")
        else:
            print(stats_str)

def main():
    """
    Main function to run the complete system
    """
    import os

    # Get API key
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set")
        sys.exit(1)

    # Create and run system
    system = CompleteVoiceToActionSystem(api_key)
    system.start()

if __name__ == "__main__":
    main()
```

## Configuration and Launch Files

### ROS2 Launch File

```python
# launch/voice_to_action.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    openai_api_key = LaunchConfiguration('openai_api_key')
    whisper_model = LaunchConfiguration('whisper_model')
    llm_model = LaunchConfiguration('llm_model')

    # Declare launch arguments
    declare_openai_api_key = DeclareLaunchArgument(
        'openai_api_key',
        default_value='',
        description='OpenAI API key for LLM processing'
    )

    declare_whisper_model = DeclareLaunchArgument(
        'whisper_model',
        default_value='base',
        description='Whisper model to use (tiny, base, small, medium, large)'
    )

    declare_llm_model = DeclareLaunchArgument(
        'llm_model',
        default_value='gpt-3.5-turbo',
        description='LLM model to use for command interpretation'
    )

    # Voice-to-action node
    voice_to_action_node = Node(
        package='my_humanoid_voice',
        executable='voice_to_action_node',
        name='voice_to_action',
        parameters=[
            {'openai_api_key': openai_api_key},
            {'whisper_model': whisper_model},
            {'llm_model': llm_model}
        ],
        remappings=[
            ('/cmd_vel', '/humanoid_robot/cmd_vel'),
            ('/joint_commands', '/humanoid_robot/joint_commands'),
            ('/tts_input', '/tts/input'),
            ('/action_status', '/voice_action/status')
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_openai_api_key,
        declare_whisper_model,
        declare_llm_model,
        voice_to_action_node
    ])
```

## Troubleshooting Common Issues

### Audio Input Issues
- **No audio detected**: Check microphone permissions and audio device settings
- **Poor audio quality**: Adjust silence thresholds and preprocessing parameters
- **High CPU usage**: Reduce audio processing frequency or use lighter models

### Whisper Issues
- **Slow transcription**: Use smaller models or optimize with fp16
- **Poor accuracy**: Fine-tune on domain-specific audio or adjust model parameters
- **Memory issues**: Process audio in chunks or use CPU-only models

### LLM Integration Issues
- **High API costs**: Implement caching and optimize prompt structure
- **Slow responses**: Use faster models or implement async processing
- **Context confusion**: Implement better context management and history tracking

### ROS2 Integration Issues
- **Action timeouts**: Adjust timeout values based on robot capabilities
- **Message queue overflow**: Implement proper buffering and rate limiting
- **Synchronization issues**: Use appropriate QoS settings and callbacks

## Summary

In this chapter, we've explored the complete voice-to-action pipeline from microphone input to robot action execution. We covered Whisper for speech recognition, LLMs for natural language understanding, and ROS2 for action execution. The system integrates audio preprocessing, speech-to-text conversion, language understanding, and robot control into a unified framework for natural human-robot interaction.

## Next Steps

- Integrate with your specific robot platform
- Fine-tune Whisper on your audio environment
- Optimize LLM prompts for your specific commands
- Implement context-aware command resolution
- Test and validate in real-world scenarios