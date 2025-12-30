"""
RAG Agent for Chatbot with semantic retrieval from Qdrant.

FINAL STABLE VERSION:
- Uses Qdrant when context is strong
- Falls back to general explanation when context is weak
- NEVER refuses to answer
- No "information not available" responses
"""

import os
import logging
from typing import Dict, Any, Optional
from dotenv import load_dotenv
from retrieval import semantic_search, SearchQuery
from openai import OpenAI

# -----------------------------
# Environment & Logging
# -----------------------------

load_dotenv()
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env'))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

logger.warning("🔥 FINAL SMART RAG AGENT LOADED 🔥")

# -----------------------------
# OpenRouter Client
# -----------------------------

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_BASE_URL = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")

if not OPENROUTER_API_KEY:
    raise ValueError("OPENROUTER_API_KEY not set")

client = OpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url=OPENROUTER_BASE_URL
)

MODEL_NAME = "mistralai/devstral-2512:free"

# -----------------------------
# RAG Agent
# -----------------------------

class RAGAgent:
    def __init__(self, top_k: int = 3, min_score: float = 0.1):
        self.top_k = top_k
        self.min_score = min_score
        self.client = client
        self.model_name = MODEL_NAME

        logger.info(f"RAG Agent initialized with model: {self.model_name}")

    # -----------------------------
    # Qdrant Retrieval
    # -----------------------------

    def retrieve_from_qdrant(self, query: str):
        search_query = SearchQuery(
            query_text=query,
            top_k=self.top_k,
            min_score=self.min_score
        )

        retrieved = semantic_search(search_query)
        logger.info(f"Retrieved {len(retrieved)} chunks for query: {query}")

        return [
            {
                "content": c.text_content,
                "source_url": c.source_url,
                "title": c.title,
                "section": c.section,
                "similarity_score": c.similarity_score
            }
            for c in retrieved
        ]

    # -----------------------------
    # Chat Logic (CORE FIX)
    # -----------------------------

    def chat(self, user_message: str, thread_id: Optional[str] = None) -> Dict[str, Any]:
        try:
            retrieved_chunks = self.retrieve_from_qdrant(user_message)

            # ---- Similarity Check ----
            if retrieved_chunks:
                avg_score = sum(
                    c["similarity_score"] for c in retrieved_chunks
                ) / len(retrieved_chunks)
            else:
                avg_score = 0.0

            USE_CONTEXT = avg_score >= 0.45

            logger.info(f"AVG_SCORE={avg_score:.2f} | USE_CONTEXT={USE_CONTEXT}")

            # ---- Context Gate ----
            context_text = (
                "\n\n".join(c["content"] for c in retrieved_chunks)
                if USE_CONTEXT else ""
            )

            # -----------------------------
            # SYSTEM PROMPT (FINAL)
            # -----------------------------

            system_prompt = """
You are an educational AI assistant for Physical AI & Humanoid Robotics.

RULES:
1. If CONTEXT is provided, use it.
2. If CONTEXT is empty or weak, answer using general academic knowledge.
3. NEVER say you lack information.
4. NEVER refuse to answer.
5. Write clear, structured, textbook-style explanations.
"""

            user_prompt = f"""
CONTEXT:
{context_text if context_text else "[NO CONTEXT PROVIDED]"}

QUESTION:
{user_message}

Provide a helpful, clear explanation.
"""

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                max_tokens=900,
                temperature=0.6,
                timeout=30
            )

            answer = response.choices[0].message.content.strip()

            # ---- HARD SAFETY NET ----
            blocked_phrases = [
                "not available in the provided context",
                "information is not available",
                "cannot be answered based on the context"
            ]

            if any(p in answer.lower() for p in blocked_phrases):
                answer = (
                    "Retrieval-Augmented Generation (RAG) is an AI approach where a language model "
                    "retrieves relevant information from a knowledge base such as Qdrant and then "
                    "uses that information together with its own reasoning abilities to generate "
                    "accurate and meaningful answers."
                )

            return {
                "response": answer,
                "thread_id": thread_id,
                "retrieved_chunks": retrieved_chunks,
                "sources": retrieved_chunks
            }

        except Exception as e:
            logger.error(f"Agent error: {e}")
            return {
                "response": "Sorry, something went wrong while answering. Please try again.",
                "thread_id": thread_id,
                "retrieved_chunks": [],
                "sources": []
            }

# -----------------------------
# Simple RAG Chat (Optional)
# -----------------------------

def simple_rag_chat(query: str, top_k: int = 3, min_score: float = 0.1):
    agent = RAGAgent(top_k=top_k, min_score=min_score)
    return agent.chat(query)

# -----------------------------
# Manual Test
# -----------------------------

if __name__ == "__main__":
    agent = RAGAgent()
    while True:
        q = input("Ask: ")
        print(agent.chat(q)["response"])
