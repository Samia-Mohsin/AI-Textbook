/**
 * Mock Chatbot Service for the Humanoid Robotics Book
 * This service simulates AI responses for the frontend
 */

interface ChatResponse {
  success: boolean;
  question: string;
  answer: string;
  sources: Array<{
    id: string;
    score: number;
    title: string;
    section: string;
  }>;
}

export class MockChatbotService {
  static async getResponse(question: string): Promise<ChatResponse> {
    // Simulate network delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    // Define some sample responses based on keywords in the question
    let answer = "I'm an AI assistant for the Humanoid Robotics Book. I can help answer questions about ROS2, simulation, AI perception, and Vision-Language-Action systems. Please ask a specific question about humanoid robotics.";

    const lowerQuestion = question.toLowerCase();

    if (lowerQuestion.includes('ros') || lowerQuestion.includes('ros2')) {
      answer = "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Key concepts include nodes, topics, services, and actions for communication between different parts of your robot system.";
    } else if (lowerQuestion.includes('gazebo') || lowerQuestion.includes('simulation')) {
      answer = "Gazebo is a 3D simulation environment for autonomous robots. It provides the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It includes realistic physics simulation, high-quality graphics, and convenient programmatic interfaces.";
    } else if (lowerQuestion.includes('nvidia') || lowerQuestion.includes('isaac')) {
      answer = "NVIDIA Isaac is a robotics platform that provides a collection of tools, libraries, and reference applications to help developers build, test, and deploy AI-based robotics applications. It includes Isaac Sim for simulation, Isaac ROS for perception and navigation, and Isaac Apps for reference applications.";
    } else if (lowerQuestion.includes('vla') || lowerQuestion.includes('vision-language') || lowerQuestion.includes('voice')) {
      answer = "Vision-Language-Action (VLA) systems enable robots to understand natural language commands, perceive their environment, and execute appropriate actions. This integration is crucial for humanoid robots operating in human environments, allowing them to respond to voice commands and perform complex tasks.";
    } else if (lowerQuestion.includes('humanoid') || lowerQuestion.includes('bipedal')) {
      answer = "Humanoid robots are robots with physical features resembling the human body, typically having a head, torso, two arms, and two legs. Developing humanoid robots involves challenges in locomotion, balance, manipulation, and human-robot interaction. Key considerations include bipedal walking algorithms, fall prevention, and natural interaction methods.";
    }

    return {
      success: true,
      question,
      answer,
      sources: [
        { id: 'mock-1', score: 0.95, title: 'Relevant Section', section: 'Module 1, Chapter 1' },
        { id: 'mock-2', score: 0.87, title: 'Related Topic', section: 'Module 3, Chapter 2' }
      ]
    };
  }
}