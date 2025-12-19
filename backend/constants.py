"""
Constants for the RAG Ingestion Pipeline
"""
from typing import Final

# Configuration values
CHUNK_SIZE: Final[int] = 512
CHUNK_OVERLAP: Final[int] = 50
RATE_LIMIT_DELAY: Final[float] = 1.0
BATCH_SIZE: Final[int] = 10

# Cohere configuration
COHERE_MODEL: Final[str] = "embed-multilingual-v3.0"
VECTOR_SIZE: Final[int] = 1024

# Qdrant configuration
QDRANT_COLLECTION_NAME: Final[str] = "rag_embeddings"
QDRANT_DISTANCE: Final[str] = "Cosine"

# Default values
DEFAULT_BOOK_BASE_URL: Final[str] = "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/"

# API Limits
COHERE_MAX_BATCH_SIZE: Final[int] = 96
MAX_RETRIES: Final[int] = 5