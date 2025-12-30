# Physical AI & Humanoid Robotics — Teaching Textbook

This repository contains a comprehensive Docusaurus-based textbook on Physical AI and Humanoid Robotics, designed to teach students and professionals the complete pipeline from robotic fundamentals to advanced Vision-Language-Action systems.

## 🤖 RAG Chatbot Integration

The textbook features an AI-powered RAG (Retrieval-Augmented Generation) chatbot that provides intelligent assistance based on the course content:

- **Backend**: FastAPI server with OpenRouter integration
- **Frontend**: React component integrated into all pages
- **Deployment**: Hugging Face Spaces for backend, GitHub Pages for frontend
- **Technology**: Qdrant vector database for semantic search

## 📚 Book Structure

The textbook is organized into 4 comprehensive modules:

### Module 1: Robotic Nervous System (ROS 2)
- **Chapter 1**: Introduction to ROS 2 Architecture
- **Chapter 2**: Writing Nodes with rclpy
- **Chapter 3**: URDF for Humanoids
- **Chapter 4**: Integrating Python Agents

### Module 2: Digital Twin (Gazebo & Unity)
- **Chapter 5**: Setting Up Gazebo
- **Chapter 6**: Sensors Simulation
- **Chapter 7**: Unity for Robotics

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- **Chapter 8**: Isaac Simulation Pipeline
- **Chapter 9**: Autonomous Navigation
- **Chapter 10**: Perception Pipeline

### Module 4: Vision-Language-Action (VLA)
- **Chapter 11**: Voice-to-Action Agents
- **Chapter 12**: Cognitive Task Planning
- **Chapter 13**: Capstone Execution

## 🎯 Capstone Project

The culmination of the textbook is an Autonomous Humanoid system that implements the complete VLA pipeline:
**VOICE ⟶ PLAN ⟶ NAVIGATE ⟶ RECOGNIZE OBJECT ⟶ MANIPULATE**

## 🛠️ Technical Architecture

### Backend Stack
- **Framework**: FastAPI
- **AI Integration**: OpenRouter API with mistralai/devstral-2512:free model
- **Vector Database**: Qdrant for semantic retrieval
- **RAG Agent**: Smart retrieval-augmented generation with fallback logic
- **Deployment**: Hugging Face Spaces (`https://kashaftariq-deploy-chatbot.hf.space`)

### Frontend Stack
- **Framework**: Docusaurus v3
- **Language**: TypeScript React
- **Chatbot**: Floating RAG assistant on all pages
- **Deployment**: GitHub Pages
- **Multi-language**: English and Urdu support (RTL/LTR)

### Key Features
- **ROS 2 Integration**: Complete robotic operating system architecture
- **Simulation Environments**: Gazebo physics and Unity visualization
- **AI Integration**: NVIDIA Isaac for GPU-accelerated perception
- **Voice Processing**: Natural language command interpretation
- **Task Planning**: Behavior trees and PDDL-based planning
- **Real-time Control**: Low-latency robotic action execution

## 🚀 Getting Started

### Local Development
1. Clone this repository
2. Navigate to the `Homanoid-Robotics-Book` directory
3. Install Docusaurus dependencies: `npm install` or `yarn install`
4. Start the development server: `npm start` or `yarn start`
5. Access the textbook at `http://localhost:3000`

### Backend Setup
1. Navigate to the `backend` directory
2. Install Python dependencies: `pip install -r requirements.txt`
3. Set environment variables in `.env` file:
   ```
   OPENROUTER_API_KEY=your_api_key
   OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=rag_embeddings
   ```
4. Start the backend: `python api.py`

## 🤖 Capstone System Requirements

- ROS 2 Humble Hawksbill
- NVIDIA GPU with CUDA support
- Gazebo Garden or newer
- Unity 2022.3 LTS or newer
- Isaac Sim and Isaac ROS packages

## 📈 Learning Outcomes

Upon completion, students will be able to:
- Design and implement complete robotic systems
- Integrate AI with physical robotic platforms
- Execute complex multi-modal tasks
- Deploy autonomous humanoid capabilities
- Evaluate and optimize system performance

## 📚 Additional Resources

- Instructor Guide with teaching recommendations
- Course Roadmap for curriculum planning
- Constitution document with project principles
- AI Notes for each module with transparency

## 🌐 Features

- **RAG Chatbot Integration**: AI-powered search and assistance
- **Optional Signup/Signin**: Personalized learning experience
- **Urdu Translation**: Multi-language support
- **Responsive Design**: Accessible on all devices
- **Interactive Examples**: Runnable code with verification

## 🏗️ Deployment Architecture

### Backend (Hugging Face Spaces)
- **URL**: `https://kashaftariq-deploy-chatbot.hf.space`
- **Technology**: FastAPI + Uvicorn
- **AI Model**: mistralai/devstral-2512:free via OpenRouter
- **Environment**: Containerized with Docker
- **Scaling**: Auto-scales based on demand

### Frontend (GitHub Pages)
- **URL**: `https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/`
- **Technology**: Docusaurus static site
- **Build Process**: GitHub Actions
- **CDN**: GitHub's global CDN
- **SSL**: Automatic HTTPS

## 📝 License

This educational content is provided for learning and academic purposes.

## 🤝 Contributing

We welcome contributions to improve and expand this textbook. Please follow the contribution guidelines in the repository.

---

*Created with ❤️ for the robotics education community*
*Version: 1.0*
*Last Updated: December 2024*