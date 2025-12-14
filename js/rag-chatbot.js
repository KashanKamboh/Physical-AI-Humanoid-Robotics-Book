// RAG Chatbot implementation for Physical AI & Humanoid Robotics Book
document.addEventListener('DOMContentLoaded', function() {
  // Create the chatbot container
  const chatbotContainer = document.createElement('div');
  chatbotContainer.id = 'rag-chatbot-container';
  document.body.appendChild(chatbotContainer);

  // Add chatbot styles if they don't exist
  if (!document.querySelector('#rag-chatbot-styles')) {
    const style = document.createElement('style');
    style.id = 'rag-chatbot-styles';
    style.textContent = `
      .rag-chatbot-container {
        position: fixed;
        bottom: 20px;
        right: 20px;
        z-index: 1000;
      }

      .rag-chatbot-button {
        background-color: #2a4d69;
        color: white;
        border: none;
        border-radius: 50%;
        width: 60px;
        height: 60px;
        font-size: 1.5rem;
        cursor: pointer;
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .rag-chatbot-panel {
        position: fixed;
        bottom: 90px;
        right: 20px;
        width: 350px;
        height: 500px;
        background-color: white;
        border-radius: 10px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        display: flex;
        flex-direction: column;
        z-index: 1000;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
        font-size: 14px;
      }

      .rag-chatbot-header {
        background-color: #2a4d69;
        color: white;
        padding: 15px;
        border-radius: 10px 10px 0 0;
        font-weight: bold;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }

      .rag-chatbot-messages {
        flex: 1;
        padding: 15px;
        overflow-y: auto;
        background-color: #f9f9f9;
      }

      .rag-chatbot-message {
        margin-bottom: 10px;
        text-align: left;
      }

      .rag-chatbot-message.user {
        text-align: right;
      }

      .rag-chatbot-message-bubble {
        display: inline-block;
        padding: 8px 12px;
        border-radius: 18px;
        max-width: 80%;
      }

      .rag-chatbot-message.user .rag-chatbot-message-bubble {
        background-color: #2a4d69;
        color: white;
      }

      .rag-chatbot-message.bot .rag-chatbot-message-bubble {
        background-color: #e9ecef;
        color: #333;
      }

      .rag-chatbot-input-form {
        display: flex;
        padding: 10px;
        border-top: 1px solid #ddd;
        background-color: white;
      }

      .rag-chatbot-input {
        flex: 1;
        padding: 10px;
        border: 1px solid #ddd;
        border-radius: 20px;
        margin-right: 10px;
      }

      .rag-chatbot-send-button {
        padding: 10px 15px;
        background-color: #2a4d69;
        color: white;
        border: none;
        border-radius: 20px;
        cursor: pointer;
      }

      .rag-chatbot-send-button:disabled {
        cursor: not-allowed;
        opacity: 0.6;
      }

      @media (max-width: 996px) {
        .rag-chatbot-container {
          bottom: 10px;
          right: 10px;
        }

        .rag-chatbot-button {
          width: 50px;
          height: 50px;
          font-size: 1.2rem;
        }

        .rag-chatbot-panel {
          width: 300px;
          height: 400px;
          bottom: 70px;
          right: 10px;
        }
      }
    `;
    document.head.appendChild(style);
  }

  // Initialize chatbot state
  let isOpen = false;
  let messages = [{
    id: 1,
    text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the course content!",
    sender: 'bot'
  }];
  let isLoading = false;

  // Create chatbot button
  const chatbotButton = document.createElement('button');
  chatbotButton.className = 'rag-chatbot-button';
  chatbotButton.textContent = '💬';
  chatbotButton.setAttribute('aria-label', 'Open chat');
  chatbotButton.onclick = toggleChat;

  chatbotContainer.appendChild(chatbotButton);

  // Create chatbot panel
  const chatbotPanel = document.createElement('div');
  chatbotPanel.className = 'rag-chatbot-panel';
  chatbotPanel.style.display = 'none';

  // Create header
  const header = document.createElement('div');
  header.className = 'rag-chatbot-header';
  header.innerHTML = `
    <span>Physical AI Assistant</span>
    <button id="rag-chatbot-close" style="background: none; border: none; color: white; font-size: 1.2rem; cursor: pointer;">×</button>
  `;
  chatbotPanel.appendChild(header);

  // Close button event
  document.getElementById('rag-chatbot-close').onclick = toggleChat;

  // Create messages container
  const messagesContainer = document.createElement('div');
  messagesContainer.className = 'rag-chatbot-messages';
  chatbotPanel.appendChild(messagesContainer);

  // Create input form
  const inputForm = document.createElement('form');
  inputForm.className = 'rag-chatbot-input-form';
  inputForm.innerHTML = `
    <input type="text" id="rag-chatbot-input" class="rag-chatbot-input" placeholder="Ask about the course..." autocomplete="off">
    <button type="submit" id="rag-chatbot-send" class="rag-chatbot-send-button">Send</button>
  `;
  chatbotPanel.appendChild(inputForm);

  // Add form event listener
  inputForm.onsubmit = function(e) {
    e.preventDefault();
    sendMessage();
  };

  chatbotContainer.appendChild(chatbotPanel);

  // Update messages display
  function updateMessagesDisplay() {
    messagesContainer.innerHTML = '';
    messages.forEach(message => {
      const messageDiv = document.createElement('div');
      messageDiv.className = `rag-chatbot-message ${message.sender}`;
      messageDiv.innerHTML = `
        <div class="rag-chatbot-message-bubble">${message.text}</div>
      `;
      messagesContainer.appendChild(messageDiv);
    });
    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  // Toggle chat visibility
  function toggleChat() {
    isOpen = !isOpen;
    chatbotPanel.style.display = isOpen ? 'flex' : 'none';
    chatbotButton.textContent = isOpen ? '×' : '💬';
    chatbotButton.setAttribute('aria-label', isOpen ? 'Close chat' : 'Open chat');

    if (isOpen) {
      document.getElementById('rag-chatbot-input').focus();
    }
  }

  // Send message
  function sendMessage() {
    const input = document.getElementById('rag-chatbot-input');
    const inputValue = input.value.trim();

    if (inputValue === '' || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };
    messages.push(userMessage);
    updateMessagesDisplay();

    // Clear input
    input.value = '';

    // Show loading state
    isLoading = true;
    updateSendButtonState();

    // Simulate bot response after delay
    setTimeout(() => {
      const botResponse = {
        id: Date.now() + 1,
        text: generateBotResponse(inputValue),
        sender: 'bot'
      };
      messages.push(botResponse);
      updateMessagesDisplay();
      isLoading = false;
      updateSendButtonState();
    }, 1000);
  }

  // Update send button state
  function updateSendButtonState() {
    const sendButton = document.getElementById('rag-chatbot-send');
    sendButton.disabled = isLoading || document.getElementById('rag-chatbot-input').value.trim() === '';
  }

  // Generate bot response based on user input
  function generateBotResponse(userInput) {
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
  }

  // Update initial display
  updateMessagesDisplay();

  // Input event listeners
  const input = document.getElementById('rag-chatbot-input');
  input.addEventListener('input', updateSendButtonState);
});

console.log('RAG Chatbot for Physical AI & Humanoid Robotics loaded');