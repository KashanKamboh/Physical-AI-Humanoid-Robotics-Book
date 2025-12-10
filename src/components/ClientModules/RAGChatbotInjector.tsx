import React, { useEffect } from 'react';
import RAGChatbot from '../RAGChatbot';

// Client module to inject the RAG chatbot into the page
const RAGChatbotInjector: React.FC = () => {
  useEffect(() => {
    // This component will be rendered once when loaded
    console.log('RAG Chatbot loaded');
  }, []);

  return <RAGChatbot />;
};

export default RAGChatbotInjector;