import React, { useState, useRef, useEffect } from 'react';
import './RAGChatbot.css';

const RAGChatbot = ({ initialContext = null }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [context, setContext] = useState(initialContext);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the RAG API
      const response = await fetch('/api/chatbot', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputValue,
          context: context
        }),
      });

      const data = await response.json();

      if (data.success) {
        // Add bot response to chat
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer,
          sender: 'bot',
          sources: data.sources,
          timestamp: new Date()
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        // Add error message
        const errorMessage = {
          id: Date.now() + 1,
          text: "Sorry, I encountered an error processing your question. Please try again.",
          sender: 'bot',
          error: true,
          timestamp: new Date()
        };

        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error calling chatbot API:', error);

      // Add error message
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I'm having trouble connecting to the service. Please try again later.",
        sender: 'bot',
        error: true,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const formatSources = (sources) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="sources">
        <strong>Sources:</strong>
        <ul>
          {sources.slice(0, 3).map((source, index) => (
            <li key={index}>
              {source.title} ({Math.round(source.score * 100)}% relevance)
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Humanoid Robotics Assistant</h3>
        <p>Ask me anything about the Humanoid Robotics Book</p>
      </div>

      <div className="chat-messages">
        {messages.length === 0 && (
          <div className="welcome-message">
            <p>Hello! I'm your Humanoid Robotics Assistant.</p>
            <p>Ask me questions about ROS2, simulation, AI perception, or Vision-Language-Action systems.</p>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.sender} ${message.error ? 'error' : ''}`}
          >
            <div className="message-content">
              <p>{message.text}</p>
              {message.sources && formatSources(message.sources)}
            </div>
            <div className="message-timestamp">
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="message bot">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about humanoid robotics..."
          rows="1"
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default RAGChatbot;