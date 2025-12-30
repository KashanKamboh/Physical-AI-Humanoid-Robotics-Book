"""
FastAPI backend service for RAG Chatbot with OpenAI Agent integration.

This module provides the API endpoints for the chatbot, integrating semantic retrieval
from Qdrant with OpenAI agent functionality.
"""

import os
import sys
import logging
from typing import Dict, List, Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

# Ensure backend directory is in path
sys.path.append(os.path.dirname(__file__))

from agent import RAGAgent, client

# Define API_TYPE for compatibility
API_TYPE = "OPENROUTER"
from retrieval import semantic_search, SearchQuery

# Load environment variables
load_dotenv()

# Logging config
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot using Qdrant retrieval and OpenRouter agent",
    version="1.0.1"
)

# CORS (open for local dev)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -----------------------------
# Pydantic Models
# -----------------------------

class ChatRequest(BaseModel):
    query: str
    top_k: int = 3
    min_score: float = 0.1
    max_tokens: int = 1000
    temperature: float = 0.7


class ChatResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    query: str
    retrieved_chunks: int


class HealthResponse(BaseModel):
    status: str
    details: Dict[str, Any]


# -----------------------------
# Global Agent
# -----------------------------

rag_agent: RAGAgent | None = None


# -----------------------------
# Startup
# -----------------------------

@app.on_event("startup")
async def startup_event():
    global rag_agent
    rag_agent = RAGAgent(top_k=3, min_score=0.1)
    logger.info("✅ RAG Agent initialized successfully")


# -----------------------------
# Routes
# -----------------------------

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running"}


@app.get("/health", response_model=HealthResponse)
async def health_check():
    try:
        # Test OpenRouter / LLM
        client.chat.completions.create(
            model="mistralai/devstral-2512:free",
            messages=[{"role": "user", "content": "health check"}],
            max_tokens=5,
            timeout=10
        )

        # Test retrieval
        test_query = SearchQuery(query_text="test", top_k=1, min_score=0.0)
        test_results = semantic_search(test_query)

        return HealthResponse(
            status="healthy",
            details={
                f"{API_TYPE.lower()}": "connected",
                "retrieval": "connected",
                "retrieval_results_count": len(test_results),
                "agent": "initialized" if rag_agent else "not_initialized"
            }
        )

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="unhealthy",
            details={"error": str(e)}
        )


@app.post("/query", response_model=ChatResponse)
async def query_endpoint(request: ChatRequest):
    """
    Main RAG chat endpoint.
    """
    try:
        if not rag_agent:
            raise HTTPException(status_code=500, detail="RAG Agent not initialized")

        logger.info(f"🧠 Query received: {request.query}")

        result = rag_agent.chat(user_message=request.query)

        answer_text = result.get("response", "").strip()
        sources = result.get("sources", [])

        if not answer_text:
            answer_text = (
                "I couldn’t find a direct answer in the course content, "
                "but you can explore the related sections listed below."
            )

        return ChatResponse(
            answer=answer_text,          # ✅ FRONTEND EXPECTS THIS
            sources=sources,
            query=request.query,
            retrieved_chunks=len(sources)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Query processing failed: {e}")
        raise HTTPException(status_code=500, detail="Error processing query")


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Alias endpoint for frontend compatibility.
    """
    return await query_endpoint(request)


# -----------------------------
# Local run
# -----------------------------

if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)
