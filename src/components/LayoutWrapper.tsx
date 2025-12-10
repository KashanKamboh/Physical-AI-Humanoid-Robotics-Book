import React from 'react';
import type { ReactNode } from 'react';
import Layout from '@theme/Layout';
import RAGChatbot from './RAGChatbot';

interface LayoutWrapperProps {
  children: ReactNode;
  title?: string;
  description?: string;
}

const LayoutWrapper: React.FC<LayoutWrapperProps> = ({ children, title, description }) => {
  return (
    <Layout title={title} description={description}>
      {children}
      <RAGChatbot />
    </Layout>
  );
};

export default LayoutWrapper;