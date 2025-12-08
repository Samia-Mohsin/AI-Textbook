import React, { useState } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './IntelligentAssistant.module.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'assistant';
  timestamp: Date;
}

const IntelligentAssistant: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for the Humanoid Robotics course. How can I help you today?',
      sender: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const { t } = useLanguage();

  const handleSendMessage = async () => {
    if (!inputText.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputText,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Use the mock service for AI response
      const data = await import('../services/mock-chatbot-service').then(mod => mod.MockChatbotService.getResponse(inputText));

      const aiResponse: Message = {
        id: (Date.now() + 1).toString(),
        text: data.answer || 'I couldn\'t process your question. Please try again.',
        sender: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, aiResponse]);
    } catch (error) {
      console.error('Error getting AI response:', error);
      const aiResponse: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, there was an error processing your question. The AI service may not be available.',
        sender: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, aiResponse]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className={styles.assistantContainer}>
      <div className={styles.assistantHeader}>
        <h3>{t('ai_assistant')}</h3>
      </div>

      <div className={styles.messagesContainer}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${styles[message.sender]}`}
          >
            <div className={styles.messageText}>{message.text}</div>
            <div className={styles.messageTime}>
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className={`${styles.message} ${styles.assistant}`}>
            <div className={styles.typingIndicator}>
              <div className={styles.dot}></div>
              <div className={styles.dot}></div>
              <div className={styles.dot}></div>
            </div>
          </div>
        )}
      </div>

      <div className={styles.inputContainer}>
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder={t('ask_robotics_question')}
          className={styles.inputField}
          rows={2}
        />
        <button
          onClick={handleSendMessage}
          disabled={isLoading || !inputText.trim()}
          className={styles.sendButton}
        >
          {t('send')}
        </button>
      </div>
    </div>
  );
};

export default IntelligentAssistant;