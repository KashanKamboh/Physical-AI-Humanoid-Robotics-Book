"""
RAG Agent for Chatbot with semantic retrieval from Qdrant.

This module implements an Agent that integrates semantic retrieval
from Qdrant to provide grounded responses based on the Physical AI & Humanoid Robotics book.
Uses Qwen model via Alibaba DashScope OpenAI-compatible mode.
"""
import os
import logging
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from dotenv import load_dotenv
import json
from retrieval import semantic_search, RetrievedChunk, SearchQuery
from openai import OpenAI

# Load environment variables - check both project root and current directory
load_dotenv()  # Load from current directory first
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env'))  # Load from project root

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize OpenAI client with DashScope
qwen_api_key = os.getenv("QWEN_API_KEY")
if not qwen_api_key:
    raise ValueError("QWEN_API_KEY environment variable not set")

client = OpenAI(
    api_key=qwen_api_key,
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
)

API_TYPE = "QWEN"
logger.info("Using Qwen model via DashScope")

class RAGAgent:
    """
    RAG Agent that performs semantic retrieval from Qdrant before generating responses.
    Supports both OpenAI and Google Gemini APIs.

    This agent follows the RAG flow:
    1. Receive user query
    2. Retrieve relevant chunks from Qdrant
    3. Generate response using LLM with retrieved context
    4. Return grounded response with source information
    """

    def __init__(self, top_k: int = 3, min_score: float = 0.1, model: str = None):
        """
        Initialize the RAG Agent.

        Args:
            top_k: Number of top results to retrieve from Qdrant
            min_score: Minimum similarity score for retrieved results
            model: Model to use for generation (defaults based on API type)
        """
        self.top_k = top_k
        self.min_score = min_score

        if API_TYPE == "QWEN":
            self.model_name = model or "qwen-plus"
            self.client = client
        elif API_TYPE == "GEMINI":
            import google.generativeai as genai
            self.model_name = model or "gemini-pro"
            genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
            self.model = genai.GenerativeModel(self.model_name)
            self.client = None  # Set to None for Gemini to avoid attribute errors
        elif API_TYPE == "OPENAI":
            self.model_name = model or "gpt-3.5-turbo"
            self.client = client

        logger.info(f"RAG Agent initialized with {API_TYPE} API and model: {self.model_name if hasattr(self, 'model_name') else model}")

    def retrieve_from_qdrant(self, query: str, top_k: int = None, min_score: float = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks from Qdrant based on the query.

        Args:
            query: Search query for semantic retrieval
            top_k: Number of results to retrieve (uses default if None)
            min_score: Minimum similarity score (uses default if None)

        Returns:
            List of retrieved chunks with metadata
        """
        if top_k is None:
            top_k = self.top_k
        if min_score is None:
            min_score = self.min_score

        search_query = SearchQuery(
            query_text=query,
            top_k=top_k,
            min_score=min_score
        )

        retrieved_chunks = semantic_search(search_query)
        logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant for query: {query[:50]}...")

        # Convert to the format expected by the agent
        formatted_chunks = []
        for chunk in retrieved_chunks:
            formatted_chunks.append({
                "content": chunk.text_content,
                "source_url": chunk.source_url,
                "title": chunk.title,
                "section": chunk.section,
                "similarity_score": chunk.similarity_score,
                "char_start": chunk.char_start,
                "char_end": chunk.char_end
            })

        return formatted_chunks

    def chat(self, user_message: str, thread_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a user message through the RAG flow.
        This method now works with both OpenAI and Gemini APIs.

        Args:
            user_message: The user's query or message
            thread_id: Optional thread ID to continue a conversation (not used in current implementation)
        Returns:
            Dictionary containing the agent's response and metadata
        """
        try:
            # Retrieve relevant chunks from Qdrant
            retrieved_chunks = self.retrieve_from_qdrant(user_message)

            # Format context from retrieved chunks
            if retrieved_chunks:
                context_texts = [chunk["content"] for chunk in retrieved_chunks]
                context = "\n\n".join(context_texts)
                full_context = f"Context from Physical AI & Humanoid Robotics book:\n{context}\n\n"
            else:
                full_context = "No relevant context found in the Physical AI & Humanoid Robotics book.\n\n"

            # Generate response based on API type
            if API_TYPE == "QWEN":
                # Create messages for Qwen (using OpenAI-compatible format)
                messages = [
                    {
                        "role": "system",
                        "content": (
                            "You are an AI assistant specialized in Physical AI & Humanoid Robotics. "
                            "Your responses must be based ONLY on the provided context from the book. "
                            "If the context doesn't contain relevant information to answer the query, "
                            "clearly state that the information is not available in the provided context. "
                            "Always be factual, concise, and cite sources when possible. "
                            "Do not hallucinate or provide information not found in the context."
                        )
                    },
                    {
                        "role": "user",
                        "content": (
                            f"{full_context}"
                            f"User query: {user_message}\n\n"
                            f"Please provide a helpful response based on the context provided, "
                            f"or state if the information is not available in the context."
                        )
                    }
                ]

                # Generate response using Qwen via DashScope
                try:
                    response = self.client.chat.completions.create(
                        model=self.model_name,
                        messages=messages,
                        max_tokens=1000,
                        temperature=0.7,
                        timeout=30
                    )

                    ai_response = response.choices[0].message.content
                except Exception as e:
                    logger.error(f"Error calling Qwen API: {str(e)}")
                    if "invalid_api_key" in str(e).lower() or "401" in str(e):
                        ai_response = "Sorry, there is an issue with the API key. Please check that a valid Qwen API key is configured."
                    else:
                        ai_response = "Sorry, I encountered an error processing your request. Please try again later."
            elif API_TYPE == "GEMINI":
                import google.generativeai as genai
                genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
                model = genai.GenerativeModel(self.model_name)

                # Create the prompt with context
                prompt = f"""{full_context}

User query: {user_message}

Please provide a helpful response based on the context provided, or state if the information is not available in the context.

You are an AI assistant specialized in Physical AI & Humanoid Robotics.
Your responses must be based ONLY on the provided context from the book.
If the context doesn't contain relevant information to answer the query,
clearly state that the information is not available in the provided context.
Always be factual, concise, and cite sources when possible.
Do not hallucinate or provide information not found in the context."""

                # Generate response using Google Gemini
                response = model.generate_content(
                    prompt,
                    generation_config={
                        "temperature": 0.7,
                        "max_output_tokens": 1000,
                    }
                )

                # Extract the response text
                if response.candidates and response.candidates[0].content.parts:
                    ai_response = response.candidates[0].content.parts[0].text
                else:
                    ai_response = "Sorry, I couldn't generate a response."
            elif API_TYPE == "OPENAI":
                # Create messages for OpenAI
                messages = [
                    {
                        "role": "system",
                        "content": (
                            "You are an AI assistant specialized in Physical AI & Humanoid Robotics. "
                            "Your responses must be based ONLY on the provided context from the book. "
                            "If the context doesn't contain relevant information to answer the query, "
                            "clearly state that the information is not available in the provided context. "
                            "Always be factual, concise, and cite sources when possible. "
                            "Do not hallucinate or provide information not found in the context."
                        )
                    },
                    {
                        "role": "user",
                        "content": (
                            f"{full_context}"
                            f"User query: {user_message}\n\n"
                            f"Please provide a helpful response based on the context provided, "
                            f"or state if the information is not available in the context."
                        )
                    }
                ]

                # Generate response using OpenAI
                response = self.client.chat.completions.create(
                    model=self.model_name,
                    messages=messages,
                    max_tokens=1000,
                    temperature=0.7,
                    timeout=30
                )

                ai_response = response.choices[0].message.content

            return {
                "response": ai_response,
                "thread_id": thread_id,
                "retrieved_chunks": retrieved_chunks,
                "sources": [chunk for chunk in retrieved_chunks]
            }

        except Exception as e:
            logger.error(f"Error in chat: {str(e)}")
            error_str = str(e).lower()
            # Check if it's an API key related error
            if "invalid_api_key" in error_str or "401" in str(e) or "authentication" in error_str or "api key" in error_str:
                # Return a specific error message for API key issues
                return {
                    "response": "Sorry, there is an issue with the API key. Please check that a valid Qwen API key is configured.",
                    "thread_id": thread_id,
                    "retrieved_chunks": [],
                    "sources": []
                }
            else:
                # Return a general error message for other issues
                return {
                    "response": "Sorry, I encountered an error processing your request. Please try again later.",
                    "thread_id": thread_id,
                    "retrieved_chunks": [],
                    "sources": []
                }

    def _wait_for_run_completion(self, thread_id: str, run_id: str) -> Any:
        """
        Wait for a run to complete, handling any required tool calls.
        """
        import time
        from openai.types.beta.threads.run import Run

        while True:
            run: Run = self.client.beta.threads.runs.retrieve(
                thread_id=thread_id,
                run_id=run_id
            )

            if run.status == "completed":
                break
            elif run.status == "failed":
                raise Exception(f"Run failed: {run.last_error}")
            elif run.status == "requires_action":
                # Handle tool calls
                tool_outputs = []

                for tool_call in run.required_action.submit_tool_outputs.tool_calls:
                    if tool_call.function.name == "retrieve_from_qdrant":
                        # Parse the function arguments
                        args = json.loads(tool_call.function.arguments)
                        query = args.get("query", "")
                        top_k = args.get("top_k", self.top_k)
                        min_score = args.get("min_score", self.min_score)

                        # Perform the retrieval
                        retrieved_chunks = self.retrieve_from_qdrant(query, top_k, min_score)

                        # Format the output for the assistant
                        output = {
                            "retrieved_chunks": len(retrieved_chunks),
                            "context": [chunk["content"] for chunk in retrieved_chunks],
                            "sources": [
                                {
                                    "title": chunk["title"],
                                    "source_url": chunk["source_url"],
                                    "section": chunk["section"],
                                    "similarity_score": chunk["similarity_score"]
                                }
                                for chunk in retrieved_chunks
                            ]
                        }

                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": json.dumps(output)
                        })

                # Submit tool outputs
                self.client.beta.threads.runs.submit_tool_outputs(
                    thread_id=thread_id,
                    run_id=run_id,
                    tool_outputs=tool_outputs
                )
            else:
                time.sleep(0.5)  # Wait before checking again

        return run

    def create_thread(self) -> str:
        """
        Create a new conversation thread.

        Returns:
            Thread ID for the new thread
        """
        thread = self.client.beta.threads.create()
        return thread.id

    def cleanup(self):
        """
        Cleanup resources, such as deleting the assistant.
        """
        try:
            self.client.beta.assistants.delete(self.assistant.id)
            logger.info(f"Assistant {self.assistant.id} deleted")
        except Exception as e:
            logger.error(f"Error deleting assistant: {str(e)}")

# Standalone function for simple RAG chat without persistent threads
def simple_rag_chat(query: str, top_k: int = 3, min_score: float = 0.1, model: str = "gpt-3.5-turbo") -> Dict[str, Any]:
    """
    Perform a simple RAG chat operation without using the full agent framework.

    Args:
        query: User query
        top_k: Number of results to retrieve
        min_score: Minimum similarity score
        model: OpenAI model to use

    Returns:
        Dictionary with response and sources
    """
    # Retrieve relevant chunks from Qdrant
    search_query = SearchQuery(
        query_text=query,
        top_k=top_k,
        min_score=min_score
    )

    retrieved_chunks = semantic_search(search_query)
    logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant")

    # Format context from retrieved chunks
    if retrieved_chunks:
        context_texts = [chunk.text_content for chunk in retrieved_chunks]
        context = "\n\n".join(context_texts)
        full_context = f"Context from Physical AI & Humanoid Robotics book:\n{context}\n\n"

        sources = [
            {
                "source_url": chunk.source_url,
                "title": chunk.title,
                "section": chunk.section,
                "similarity_score": chunk.similarity_score,
                "char_start": chunk.char_start,
                "char_end": chunk.char_end
            }
            for chunk in retrieved_chunks
        ]
    else:
        full_context = "No relevant context found in the Physical AI & Humanoid Robotics book.\n\n"
        sources = []

    # Create messages for OpenAI
    messages = [
        {
            "role": "system",
            "content": (
                "You are an AI assistant specialized in Physical AI & Humanoid Robotics. "
                "Your responses must be based ONLY on the provided context from the book. "
                "If the context doesn't contain relevant information to answer the query, "
                "clearly state that the information is not available in the provided context. "
                "Always be factual, concise, and cite sources when possible. "
                "Do not hallucinate or provide information not found in the context."
            )
        },
        {
            "role": "user",
            "content": (
                f"{full_context}"
                f"User query: {query}\n\n"
                f"Please provide a helpful response based on the context provided, "
                f"or state if the information is not available in the context."
            )
        }
    ]

    # Generate response using the appropriate API based on environment configuration
    if API_TYPE == "QWEN":
        # Generate response using Qwen via DashScope
        try:
            response = client.chat.completions.create(
                model="qwen-plus",  # Use the Qwen model
                messages=messages,
                max_tokens=1000,
                temperature=0.7,
                timeout=30
            )
            ai_response = response.choices[0].message.content
        except Exception as e:
            logger.error(f"Error calling Qwen API in simple_rag_chat: {str(e)}")
            if "invalid_api_key" in str(e).lower() or "401" in str(e):
                ai_response = "Sorry, there is an issue with the API key. Please check that a valid Qwen API key is configured."
            else:
                ai_response = "Sorry, I encountered an error processing your request. Please try again later."
    elif API_TYPE == "GEMINI":
        import google.generativeai as genai
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        gemini_model = genai.GenerativeModel('gemini-pro')

        # Create the prompt with context
        prompt = f"""{full_context}

User query: {query}

Please provide a helpful response based on the context provided, or state if the information is not available in the context.

You are an AI assistant specialized in Physical AI & Humanoid Robotics.
Your responses must be based ONLY on the provided context from the book.
If the context doesn't contain relevant information to answer the query,
clearly state that the information is not available in the provided context.
Always be factual, concise, and cite sources when possible.
Do not hallucinate or provide information not found in the context."""

        # Generate response using Google Gemini
        gemini_response = gemini_model.generate_content(
            prompt,
            generation_config={
                "temperature": 0.7,
                "max_output_tokens": 1000,
            }
        )

        # Extract the response text
        if gemini_response.candidates and gemini_response.candidates[0].content.parts:
            ai_response = gemini_response.candidates[0].content.parts[0].text
        else:
            ai_response = "Sorry, I couldn't generate a response."
    else:  # OPENAI
        from openai import OpenAI  # Import here in case it wasn't imported globally
        client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=1000,
            temperature=0.7,
            timeout=30
        )
        ai_response = response.choices[0].message.content

    return {
        "response": ai_response,
        "sources": sources,
        "query": query,
        "retrieved_chunks": len(retrieved_chunks)
    }

if __name__ == "__main__":
    # Example usage
    agent = RAGAgent(top_k=3, min_score=0.1)

    try:
        # Test the agent
        result = agent.chat("What is the robotic nervous system?")
        print("Response:", result["response"])
        print("Sources:", result["sources"])
    except Exception as e:
        print(f"Error: {e}")
    finally:
        agent.cleanup()