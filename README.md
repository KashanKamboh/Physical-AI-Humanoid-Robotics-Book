# RAG Chatbot - Physical AI & Humanoid Robotics

This project implements a RAG-based AI chatbot using book content stored in Qdrant, with a FastAPI + OpenAI Agent backend and React/Docusaurus frontend.

## Architecture

The system consists of three main components:

1. **FastAPI Backend Service**: Exposes chatbot query endpoints with OpenAI Agent integration
2. **OpenAI Agent**: Performs semantic retrieval from Qdrant before generating responses
3. **React/Docusaurus Frontend**: Chatbot UI integrated on every page of the book

## Prerequisites

- Python 3.8+
- Node.js 16+
- OpenAI API key
- Qdrant vector database (already configured)

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install Python dependencies
pip install fastapi uvicorn python-dotenv openai qdrant-client cohere pydantic requests

# Create .env file with your API keys
cp .env.example .env
# Edit .env to add your OpenAI API key
```

### 3. Frontend Setup

```bash
# Navigate to the Docusaurus directory
cd Homanoid-Robotics-Book

# Install dependencies
npm install
```

## Running the System

### 1. Start the Backend Service

```bash
# From the project root directory
python start_backend.py
```

The backend will be available at `http://localhost:8000`

### 2. Start the Frontend (Docusaurus)

```bash
# From the Homanoid-Robotics-Book directory
npm start
```

The frontend will be available at `http://localhost:3000`

## Testing the RAG Flow

You can test the complete RAG flow using the test script:

```bash
python test_rag_flow.py
```

This will:
- Check the health of the backend service
- Send sample queries to test the RAG pipeline
- Verify that semantic retrieval is working
- Confirm that responses are grounded in book content

## API Endpoints

- `GET /health` - Health check
- `POST /query` - Main chatbot query endpoint
- `POST /chat` - Alternative chat endpoint (same functionality as /query)

### Query Request Format

```json
{
  "query": "Your question here",
  "top_k": 3,
  "min_score": 0.1,
  "max_tokens": 1000,
  "temperature": 0.7
}
```

### Query Response Format

```json
{
  "response": "AI-generated response",
  "sources": [
    {
      "source_url": "URL of source",
      "title": "Title of section",
      "section": "Section name",
      "similarity_score": 0.85,
      "char_start": 0,
      "char_end": 100
    }
  ],
  "query": "Original query",
  "retrieved_chunks": 3
}
```

## RAG Flow

The system follows this RAG flow:

1. User submits a query from the frontend
2. Backend retrieves relevant chunks from **Qdrant**
3. Retrieved context + user query is passed to the **Agent**
4. Agent generates a **deterministic, grounded response**
5. Response is returned and rendered in the chat UI

## Frontend Integration

The chatbot UI is integrated into the Docusaurus layout and appears on every page of the book. The React component:

- Collects user input
- Calls the backend RAG API
- Renders the returned response and sources
- Maintains the existing UI/UX

## Environment Variables

Create a `.env` file in the project root with:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here  # Optional, defaults to localhost
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional
COHERE_API_KEY=your_cohere_api_key_here  # Optional
PORT=8000  # Backend port, defaults to 8000
```

## Development

### Running in Development Mode

The backend starts with auto-reload enabled. The frontend also has hot-reloading.

### Testing Individual Components

- Backend: `python -m pytest tests/` (if tests exist)
- Frontend: `npm run build` to test build process

## Deployment

### Backend (to Railway/Heroku/etc.)

1. Set environment variables in your deployment platform
2. Deploy the backend service
3. Note the backend URL for frontend configuration

### Frontend (to GitHub Pages/Netlify/Vercel)

1. Update the backend URL in the frontend configuration
2. Build and deploy the Docusaurus site

## Troubleshooting

- **Backend not starting**: Check that all environment variables are set
- **Qdrant connection issues**: Verify Qdrant is running and accessible
- **Frontend can't connect to backend**: Check CORS settings and backend URL
- **No responses**: Verify that the book content has been properly ingested into Qdrant

## Security

- All API keys are stored in environment variables
- No secrets are stored in the frontend code
- CORS is configured to allow necessary origins