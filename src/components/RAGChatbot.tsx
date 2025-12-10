import React, { useState, useEffect } from 'react';
import type { ReactNode } from 'react';

// RAG Chatbot Component for the Physical AI & Humanoid Robotics Book
const RAGChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Array<{id: number, text: string, sender: 'user' | 'bot'}>>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Initialize with a welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        {
          id: 1,
          text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the course content!",
          sender: 'bot'
        }
      ]);
    }
  }, [messages.length]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() === '' || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user' as const
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    // Simulate bot response after delay
    setTimeout(() => {
      const botResponse = {
        id: Date.now() + 1,
        text: generateBotResponse(inputValue),
        sender: 'bot' as const
      };
      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
    }, 1000);
  };

  const generateBotResponse = (userInput: string): string => {
    const input = userInput.toLowerCase();

    if (input.includes('hello') || input.includes('hi') || input.includes('hey')) {
      return "Hello there! I'm here to help you with the Physical AI & Humanoid Robotics course. You can ask me about ROS 2, Gazebo, Unity, NVIDIA Isaac, or any other topic from the curriculum.";
    } else if (input.includes('module') || input.includes('chapter')) {
      return "The course has 4 comprehensive modules: 1) Robotic Nervous System (ROS 2), 2) Digital Twin (Gazebo & Unity), 3) AI-Robot Brain (NVIDIA Isaac), and 4) Vision-Language-Action (VLA). Which module would you like to know more about?";
    } else if (input.includes('ros') || input.includes('ros 2')) {
      return "ROS 2 (Robot Operating System 2) is covered in Module 1. It provides the communication infrastructure for robotics applications. Key topics include nodes, topics, services, actions, and launch files. Would you like specific information about any ROS 2 concept?";
    } else if (input.includes('gazebo') || input.includes('simulation')) {
      return "Gazebo simulation is covered in Module 2. It provides physics-based 3D simulation for robotics. You'll learn about world creation, robot models, sensor simulation, and realistic physics properties. The course covers both basic and advanced simulation techniques.";
    } else if (input.includes('unity') || input.includes('visualization')) {
      return "Unity for robotics is also covered in Module 2. It provides advanced visualization and interaction capabilities for robotics applications. You'll learn about humanoid rig setup, motion blending, ray-interaction events, and integration with ROS 2.";
    } else if (input.includes('isaac') || input.includes('nvidia')) {
      return "NVIDIA Isaac is covered in Module 3. It provides GPU-accelerated robotics tools including Isaac Sim for advanced simulation, Isaac ROS for perception, and integration with Nav2 for navigation. The focus is on AI-robot brain capabilities.";
    } else if (input.includes('vla') || input.includes('vision-language-action')) {
      return "Vision-Language-Action (VLA) systems are covered in Module 4. This includes voice-to-action agents, cognitive task planning, and the complete pipeline: VOICE ⟶ PLAN ⟶ NAVIGATE ⟶ RECOGNIZE OBJECT ⟶ MANIPULATE.";
    } else if (input.includes('capstone') || input.includes('project')) {
      return "The capstone project integrates all modules into an autonomous humanoid system. It demonstrates the complete VLA pipeline with voice commands, planning, navigation, object recognition, and manipulation. The project showcases all learned concepts in a real-world application.";
    } else {
      return "I'm here to help you with the Physical AI & Humanoid Robotics course. You can ask me about specific modules, chapters, code examples, or concepts from the curriculum. For example, you could ask about ROS 2, Gazebo, Unity, NVIDIA Isaac, or the capstone project.";
    }
  };

  return (
    <>
      {/* Chatbot button */}
      <div className="rag-chatbot-container">
        <button
          className="rag-chatbot-button"
          onClick={toggleChat}
          aria-label={isOpen ? "Close chat" : "Open chat"}
        >
          {isOpen ? '×' : '💬'}
        </button>
      </div>

      {/* Chatbot panel */}
      {isOpen && (
        <div className="rag-chatbot-panel" style={{
          position: 'fixed',
          bottom: '90px',
          right: '20px',
          width: '350px',
          height: '500px',
          backgroundColor: 'white',
          borderRadius: '10px',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          display: 'flex',
          flexDirection: 'column',
          zIndex: 1000,
          fontFamily: 'var(--ifm-font-family-base)',
          fontSize: 'var(--ifm-font-size-base)'
        }}>
          {/* Header */}
          <div style={{
            backgroundColor: '#2a4d69',
            color: 'white',
            padding: '15px',
            borderRadius: '10px 10px 0 0',
            fontWeight: 'bold',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <span>Physical AI Assistant</span>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '1.2rem',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div style={{
            flex: 1,
            padding: '15px',
            overflowY: 'auto',
            backgroundColor: '#f9f9f9'
          }}>
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  marginBottom: '10px',
                  textAlign: message.sender === 'user' ? 'right' : 'left'
                }}
              >
                <div style={{
                  display: 'inline-block',
                  padding: '8px 12px',
                  borderRadius: '18px',
                  backgroundColor: message.sender === 'user' ? '#2a4d69' : '#e9ecef',
                  color: message.sender === 'user' ? 'white' : '#333',
                  maxWidth: '80%'
                }}>
                  {message.text}
                </div>
              </div>
            ))}
            {isLoading && (
              <div style={{ textAlign: 'left', marginBottom: '10px' }}>
                <div style={{
                  display: 'inline-block',
                  padding: '8px 12px',
                  borderRadius: '18px',
                  backgroundColor: '#e9ecef',
                  color: '#333'
                }}>
                  Thinking...
                </div>
              </div>
            )}
          </div>

          {/* Input */}
          <form onSubmit={handleSendMessage} style={{
            display: 'flex',
            padding: '10px',
            borderTop: '1px solid #ddd',
            backgroundColor: 'white'
          }}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about the course..."
              style={{
                flex: 1,
                padding: '10px',
                border: '1px solid #ddd',
                borderRadius: '20px',
                marginRight: '10px'
              }}
              disabled={isLoading}
            />
            <button
              type="submit"
              style={{
                padding: '10px 15px',
                backgroundColor: '#2a4d69',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: isLoading ? 'not-allowed' : 'pointer'
              }}
              disabled={isLoading || inputValue.trim() === ''}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default RAGChatbot;