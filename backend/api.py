"""
FastAPI backend service for RAG Chatbot with OpenAI Agent integration.

This module provides the API endpoints for the chatbot, integrating semantic retrieval
from Qdrant with OpenAI agent functionality.
"""
import os
import logging
from typing import Dict, List, Any
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import sys
import os
sys.path.append(os.path.dirname(__file__))  # Add the backend directory to Python path

from agent import RAGAgent, simple_rag_chat
from retrieval import semantic_search, SearchQuery
from openai import OpenAI

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot using Qdrant retrieval and OpenAI agent",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for request/response
class ChatRequest(BaseModel):
    query: str
    top_k: int = 3
    min_score: float = 0.1
    max_tokens: int = 1000
    temperature: float = 0.7

class ChatResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    query: str
    retrieved_chunks: int

class HealthResponse(BaseModel):
    status: str
    details: Dict[str, Any]

# Determine which API to use based on available keys (support Qwen, OpenAI, and Gemini)
qwen_api_key = os.getenv("QWEN_API_KEY")
openai_api_key = os.getenv("OPENAI_API_KEY")
gemini_api_key = os.getenv("GEMINI_API_KEY")

if qwen_api_key:
    # Use Qwen model via Alibaba DashScope
    from openai import OpenAI
    client = OpenAI(
        api_key=qwen_api_key,
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
    )
    API_TYPE = "QWEN"
    logger.info("Using Qwen model via DashScope")
elif gemini_api_key:
    # Use Google Gemini API
    import google.generativeai as genai
    genai.configure(api_key=gemini_api_key)
    gemini_model = genai.GenerativeModel('gemini-pro')
    API_TYPE = "GEMINI"
    logger.info("Using Google Gemini API")
elif openai_api_key:
    # Use OpenAI API
    from openai import OpenAI
    client = OpenAI(api_key=openai_api_key)
    API_TYPE = "OPENAI"
    logger.info("Using OpenAI API")
else:
    raise ValueError("Either QWEN_API_KEY, OPENAI_API_KEY, or GEMINI_API_KEY must be set")

# Global agent instance
rag_agent = None

# Initialize the RAG agent on startup
@app.on_event("startup")
async def startup_event():
    global rag_agent
    rag_agent = RAGAgent(top_k=3, min_score=0.1)
    logger.info("RAG Agent initialized on startup")

@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "RAG Chatbot API is running"}

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint to verify service status."""
    try:
        # Test the appropriate API based on configuration
        if API_TYPE == "QWEN":
            try:
                test_completion = client.chat.completions.create(
                    model="qwen-plus",
                    messages=[{"role": "user", "content": "test"}],
                    max_tokens=5,
                    timeout=10
                )
                api_status = "qwen_connected"
            except Exception as e:
                if "invalid_api_key" in str(e).lower() or "401" in str(e):
                    logger.error("Invalid Qwen API key provided")
                    return HealthResponse(
                        status="unhealthy",
                        details={
                            "qwen": "invalid_api_key",
                            "retrieval": "connected",
                            "retrieval_results_count": 0,
                            "agent": "initialized"
                        }
                    )
                else:
                    raise e
        elif API_TYPE == "GEMINI":
            import google.generativeai as genai
            genai.configure(api_key=gemini_api_key)
            test_model = genai.GenerativeModel('gemini-pro')
            test_response = test_model.generate_content("test", generation_config={"max_output_tokens": 5})
            api_status = "gemini_connected"
        elif API_TYPE == "OPENAI":
            test_completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5,
                timeout=10
            )
            api_status = "openai_connected"

        # Test retrieval system
        test_query = SearchQuery(query_text="test", top_k=1, min_score=0.0)
        test_results = semantic_search(test_query)

        # Test agent initialization
        agent_status = "initialized" if rag_agent else "not initialized"

        return HealthResponse(
            status="healthy",
            details={
                f"{API_TYPE.lower()}": "connected",
                "retrieval": "connected",
                "retrieval_results_count": len(test_results),
                "agent": agent_status
            }
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            details={"error": str(e)}
        )

@app.post("/query", response_model=ChatResponse)
async def query_endpoint(request: ChatRequest):
    """
    Main endpoint for chatbot queries.

    This endpoint uses the RAG Agent to perform semantic retrieval from Qdrant,
    then passes the retrieved context along with the user query to an OpenAI agent for
    response generation.
    """
    try:
        logger.info(f"Processing query: {request.query[:100]}...")

        # Check if the agent is initialized
        if not rag_agent:
            raise HTTPException(status_code=500, detail="RAG Agent not initialized")

        # Use the RAGAgent which supports both OpenAI and Gemini APIs
        result = rag_agent.chat(user_message=request.query)

        # Check if the result contains an error message due to API key issues
        if result.get("response") and ("API key" in result["response"] or "invalid_api_key" in result["response"] or "401" in result["response"]):
            raise HTTPException(status_code=401, detail="Invalid API key. Please check your API key configuration.")

        # Format the result to match the expected response structure
        formatted_result = {
            "response": result["response"],
            "sources": result["sources"],
            "query": request.query,
            "retrieved_chunks": len(result["sources"])
        }

        logger.info(f"Generated response with {len(formatted_result['response'])} characters")

        # Return the response with sources
        return ChatResponse(
            response=formatted_result["response"],
            sources=formatted_result["sources"],
            query=formatted_result["query"],
            retrieved_chunks=formatted_result["retrieved_chunks"]
        )

    except HTTPException:
        # Re-raise HTTP exceptions (like 401 for API key issues)
        raise
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        # Check if it's an API key related error
        error_str = str(e).lower()
        if "invalid_api_key" in error_str or "401" in str(e) or "authentication" in error_str:
            raise HTTPException(status_code=401, detail="Invalid API key. Please check your API key configuration.")
        else:
            raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Alternative chat endpoint with the same functionality as /query.
    This provides compatibility with different frontend implementations.
    """
    return await query_endpoint(request)

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)