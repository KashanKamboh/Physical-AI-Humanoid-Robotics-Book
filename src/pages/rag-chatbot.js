// Client module to add RAG chatbot to all pages
import React from 'react';
import RAGChatbot from '../components/RAGChatbot';

// This component will be rendered once and persist across page navigations
const RAGChatbotModule = () => {
  return <RAGChatbot />;
};

export default RAGChatbotModule;