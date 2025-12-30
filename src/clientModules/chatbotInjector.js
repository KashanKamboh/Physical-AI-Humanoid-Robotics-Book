// Client module to inject the RAG chatbot
import { createRoot } from 'react-dom/client';
import React from 'react';
import RAGChatbot from '../components/RAGChatbot';

// Create and inject the chatbot component
function injectChatbot() {
  // Create a container for the chatbot
  const chatbotContainer = document.createElement('div');
  chatbotContainer.id = 'rag-chatbot-container';
  document.body.appendChild(chatbotContainer);

  // Render the chatbot
  const root = createRoot(chatbotContainer);
  root.render(<RAGChatbot />);
}

// Wait for the DOM to be ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', injectChatbot);
} else {
  injectChatbot();
}

// Export as a client module
export default function ChatbotModule() {
  return null; // This component doesn't render anything itself
}