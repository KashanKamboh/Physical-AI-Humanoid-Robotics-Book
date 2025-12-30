import React, { useState, useEffect } from 'react';

// -----------------------------
// Types
// -----------------------------

interface ChatRequest {
  query: string;
  top_k?: number;
  min_score?: number;
  max_tokens?: number;
  temperature?: number;
}

interface ChatResponse {
  answer?: string;
  response?: string;
  sources?: Array<{
    source_url: string;
    title: string;
    section: string;
    similarity_score: number;
    char_start: number;
    char_end: number;
  }>;
  query?: string;
  retrieved_chunks?: number;
}

// -----------------------------
// Component
// -----------------------------

const RAGChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<
    Array<{ id: number; text: string; sender: 'user' | 'bot' }>
  >([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        {
          id: 1,
          text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the course content!",
          sender: 'bot',
        },
      ]);
    }
  }, [messages.length]);

  const toggleChat = () => setIsOpen(!isOpen);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userText = inputValue;

    // Add user message
    setMessages(prev => [
      ...prev,
      { id: Date.now(), text: userText, sender: 'user' },
    ]);

    setInputValue('');
    setIsLoading(true);

    try {
      // ✅ Docusaurus-safe backend URL
      const BACKEND_URL =
        window.location.hostname === 'localhost'
          ? 'http://localhost:8000'
          : 'https://kashaftariq-deploy-chatbot.hf.space';

      const res = await fetch(`${BACKEND_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: userText,
          top_k: 3,
          min_score: 0.1,
          max_tokens: 1000,
          temperature: 0.7,
        } as ChatRequest),
      });

      if (!res.ok) {
        throw new Error(`API failed with status ${res.status}`);
      }

      const data: ChatResponse = await res.json();

      const botText =
        data.answer ||
        data.response ||
        "I couldn't find a direct answer in the course content, but related material may still be helpful.";

      setMessages(prev => [
        ...prev,
        { id: Date.now() + 1, text: botText, sender: 'bot' },
      ]);
    } catch (err) {
      console.error('Chatbot API error:', err);

      setMessages(prev => [
        ...prev,
        {
          id: Date.now() + 1,
          text:
            "I'm having trouble answering right now. Please try again in a moment.",
          sender: 'bot',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Floating Button */}
      <div className="rag-chatbot-container">
        <button
          className="rag-chatbot-button"
          onClick={toggleChat}
          aria-label={isOpen ? 'Close chat' : 'Open chat'}
        >
          {isOpen ? '×' : '💬'}
        </button>
      </div>

      {/* Chat Panel */}
      {isOpen && (
        <div
          className="rag-chatbot-panel"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '350px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 10px 30px rgba(0,0,0,0.2)',
            display: 'flex',
            flexDirection: 'column',
            zIndex: 1000,
            fontFamily: 'var(--ifm-font-family-base)',
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: '#1e3a5f',
              color: 'white',
              padding: '14px',
              borderRadius: '12px 12px 0 0',
              fontWeight: 600,
              display: 'flex',
              justifyContent: 'space-between',
            }}
          >
            <span>Physical AI Assistant</span>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '1.2rem',
                cursor: 'pointer',
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div
            style={{
              flex: 1,
              padding: '14px',
              overflowY: 'auto',
              backgroundColor: '#f8fafc',
            }}
          >
            {messages.map(msg => (
              <div
                key={msg.id}
                style={{
                  marginBottom: '10px',
                  textAlign: msg.sender === 'user' ? 'right' : 'left',
                }}
              >
                <div
                  style={{
                    display: 'inline-block',
                    padding: '8px 12px',
                    borderRadius: '18px',
                    backgroundColor:
                      msg.sender === 'user' ? '#1e3a5f' : '#e5e7eb',
                    color: msg.sender === 'user' ? 'white' : '#111827',
                    maxWidth: '80%',
                  }}
                >
                  {msg.text}
                </div>
              </div>
            ))}

            {isLoading && <div style={{ color: '#555' }}>Thinking…</div>}
          </div>

          {/* Input */}
          <form
            onSubmit={handleSendMessage}
            style={{
              display: 'flex',
              padding: '10px',
              borderTop: '1px solid #e5e7eb',
            }}
          >
            <input
              type="text"
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              placeholder="Ask about the course..."
              style={{
                flex: 1,
                padding: '10px',
                borderRadius: '20px',
                border: '1px solid #d1d5db',
                marginRight: '8px',
              }}
              disabled={isLoading}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              style={{
                padding: '10px 16px',
                backgroundColor: '#1e3a5f',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: 'pointer',
              }}
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