import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import useTextSelection from '../../hooks/useTextSelection';

const ChatInterface = ({ selectedText: propSelectedText = '', onTextSelected }) => {
  const { selectedText: hookSelectedText } = useTextSelection();
  // Use propSelectedText if provided, otherwise use the hook's selected text
  const currentSelectedText = propSelectedText || hookSelectedText;
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Safely get user from context, defaulting to null if context is not available
  let user;
  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  // Add initial context if selected text is available
  useEffect(() => {
    if (currentSelectedText && messages.length === 0) {
      setMessages([{
        id: 'context',
        content: `Selected text: "${currentSelectedText}"\n\nYou can ask me about this content.`,
        sender: 'system',
        timestamp: new Date().toISOString()
      }]);
    }
  }, [currentSelectedText]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now().toString(),
      content: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    // Add user message to chat
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const requestBody = {
        message: inputValue,
        selected_text: currentSelectedText,
        user_id: user?.id,
        session_id: sessionId
      };

      // Call the API
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth_token')}`
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Update session ID if new one is provided
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add AI response to chat
      const aiMessage = {
        id: `ai-${Date.now()}`,
        content: data.response,
        sender: 'assistant',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: `error-${Date.now()}`,
        content: 'Sorry, I encountered an error. Please try again.',
        sender: 'system',
        timestamp: new Date().toISOString()
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

  const formatTime = (timestamp) => {
    return new Date(timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-grow overflow-y-auto p-4 bg-gray-50 dark:bg-gray-700">
        {messages.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-full text-gray-500 dark:text-gray-400">
            <p>Ask me anything about the Physical AI & Humanoid Robotics content!</p>
            {currentSelectedText && (
              <p className="mt-2 text-sm italic">Selected text: "{currentSelectedText.substring(0, 50)}..."</p>
            )}
          </div>
        ) : (
          <div className="space-y-4">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`flex ${message.sender === 'user' ? 'justify-end' : 'justify-start'}`}
              >
                <div
                  className={`max-w-[80%] rounded-lg p-3 ${
                    message.sender === 'user'
                      ? 'bg-blue-500 text-white'
                      : message.sender === 'system'
                      ? 'bg-yellow-100 dark:bg-yellow-900 text-yellow-800 dark:text-yellow-200'
                      : 'bg-gray-200 dark:bg-gray-600 text-gray-800 dark:text-gray-200'
                  }`}
                >
                  <div className="whitespace-pre-wrap">{message.content}</div>
                  <div className="text-xs opacity-70 mt-1">
                    {formatTime(message.timestamp)}
                  </div>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="flex justify-start">
                <div className="bg-gray-200 dark:bg-gray-600 text-gray-800 dark:text-gray-200 rounded-lg p-3 max-w-[80%]">
                  <div className="flex space-x-2">
                    <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce"></div>
                    <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
                    <div className="w-2 h-2 bg-gray-500 rounded-full animate-bounce" style={{ animationDelay: '0.4s' }}></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      <form onSubmit={handleSubmit} className="border-t border-gray-200 dark:border-gray-700 p-3">
        <div className="flex">
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={currentSelectedText ? "Ask about selected text..." : "Type your message..."}
            className="flex-grow border border-gray-300 dark:border-gray-600 rounded-l-lg px-3 py-2 focus:outline-none focus:ring-2 focus:ring-blue-500 dark:bg-gray-800 dark:text-white"
            disabled={isLoading}
          />
          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading}
            className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-r-lg disabled:opacity-50 disabled:cursor-not-allowed"
          >
            Send
          </button>
        </div>
      </form>
    </div>
  );
};

export default ChatInterface;