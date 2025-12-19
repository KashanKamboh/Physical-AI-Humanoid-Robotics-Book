"""
Semantic search implementation for RAG Chatbot retrieval.

This module provides functionality to retrieve relevant text chunks from the
Physical AI & Humanoid Robotics book using semantic search against the Qdrant vector database.
It leverages Cohere's embedding model to generate query embeddings and performs
cosine similarity search in the vector database to find semantically similar content.

The module includes comprehensive error handling, validation, and performance logging
to ensure reliable retrieval operations.
"""
import logging
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from functools import lru_cache
from config.settings import get_qdrant_client, get_cohere_client, QDRANT_COLLECTION_NAME
from retrieval.models import RetrievedChunk, SearchQuery, ValidationResult
from retrieval.exceptions import QdrantConnectionError, CohereAPIError, QueryValidationError
from retrieval.utils import sort_chunks_by_similarity, filter_by_similarity_threshold


# Simple in-memory cache for query results (in production, use Redis or similar)
_query_cache = {}

def semantic_search(query: SearchQuery) -> List[RetrievedChunk]:
    """
    Perform semantic search on embedded book content using vector similarity.

    This function takes a search query, generates an embedding using Cohere's API,
    and performs a cosine similarity search against the Qdrant vector database
    to find the most semantically similar text chunks from the Physical AI &
    Humanoid Robotics book.

    The function includes comprehensive validation, error handling, and performance
    logging to ensure reliable retrieval operations. It handles connection issues,
    query validation, and applies filters based on similarity thresholds.
    Performance is optimized through result caching and efficient filtering.

    Args:
        query: SearchQuery object containing the query text and parameters
               including top_k (number of results to return) and min_score
               (minimum similarity threshold)

    Returns:
        List of RetrievedChunk objects sorted by similarity score in descending order.
        Each chunk contains the text content, similarity score, and metadata.

    Raises:
        QueryValidationError: If query parameters are invalid
        QdrantConnectionError: If there's a connection issue with Qdrant
        CohereAPIError: If there's an issue with the Cohere API
    """
    # Validate query parameters
    if not query.query_text.strip():
        raise QueryValidationError("Query text cannot be empty")

    if not 1 <= query.top_k <= 10:
        raise QueryValidationError(f"top_k must be between 1 and 10, got {query.top_k}")

    if not 0 <= query.min_score <= 1:
        raise QueryValidationError(f"min_score must be between 0 and 1, got {query.min_score}")

    # Create cache key
    cache_key = f"{query.query_text}:{query.top_k}:{query.min_score}"

    # Check cache first
    if cache_key in _query_cache:
        logging.info(f"Cache hit for query: '{query.query_text[:50]}...'")
        return _query_cache[cache_key]

    try:
        # Get clients
        qdrant_client = get_qdrant_client()
        cohere_client = get_cohere_client()

        # Generate embedding for the query
        response = cohere_client.embed(
            texts=[query.query_text],
            model="embed-multilingual-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Validate embedding was generated properly
        if not query_embedding or len(query_embedding) != 1024:
            raise CohereAPIError(f"Invalid embedding generated: expected 1024 dimensions, got {len(query_embedding) if query_embedding else 0}")

        # Perform search in Qdrant with optimized parameters
        search_results = qdrant_client.query_points(
            collection_name=QDRANT_COLLECTION_NAME,
            query=query_embedding,
            limit=query.top_k * 2,  # Get more results to account for filtering
            with_payload=True,
            with_vectors=False,
            score_threshold=query.min_score
        )

        # Convert search results to RetrievedChunk objects with optimized processing and validation
        retrieved_chunks = []
        for result in search_results.points:
            payload = result.payload
            # Direct assignment to avoid multiple dict lookups
            text_content = payload.get("text", "")
            # Validate payload data before creating RetrievedChunk
            source_url = payload.get("source_url", "")
            title = payload.get("title", "")
            section = payload.get("section", "")
            created_at = payload.get("created_at", "")
            char_start = payload.get("char_start", 0)
            char_end = payload.get("char_end", 0)

            # Validate required fields have appropriate types/values
            if not isinstance(char_start, int) or not isinstance(char_end, int):
                logging.warning(f"Invalid char positions for result {result.id}, skipping")
                continue

            if char_start >= char_end:
                logging.warning(f"Invalid char range for result {result.id} ({char_start} >= {char_end}), skipping")
                continue

            # Skip chunks with empty text content as they're not useful for retrieval
            if not text_content:
                logging.warning(f"Skipping result {result.id} with empty text content")
                continue

            chunk = RetrievedChunk(
                chunk_id=str(result.id),
                text_content=text_content,
                similarity_score=result.score,
                source_url=source_url,
                title=title,
                section=section,
                created_at=created_at,
                char_start=char_start,
                char_end=char_end
            )
            retrieved_chunks.append(chunk)

        # Filter by minimum score if needed (optimized by checking threshold first)
        if query.min_score > 0 and retrieved_chunks:
            retrieved_chunks = filter_by_similarity_threshold(retrieved_chunks, query.min_score)

        # Sort by similarity score (descending) - only if we have more than 1 result
        if len(retrieved_chunks) > 1:
            retrieved_chunks = sort_chunks_by_similarity(retrieved_chunks)

        # Limit to top_k results
        retrieved_chunks = retrieved_chunks[:query.top_k]

        # Validate final results before returning
        for chunk in retrieved_chunks:
            if not isinstance(chunk, RetrievedChunk):
                raise ValueError(f"Invalid chunk type in results: {type(chunk)}")
            # Additional validation is handled by the dataclass's __post_init__

        # Cache the results (with size limit to prevent memory issues)
        if len(_query_cache) < 100:  # Limit cache size
            _query_cache[cache_key] = retrieved_chunks

        # Log search operation
        logging.info(f"Semantic search completed for query: '{query.query_text[:50]}...' "
                    f"Retrieved {len(retrieved_chunks)} chunks")

        return retrieved_chunks

    except Exception as e:
        logging.error(f"Error during semantic search: {str(e)}")
        if "qdrant" in str(e).lower():
            raise QdrantConnectionError(f"Qdrant connection error: {str(e)}")
        elif "cohere" in str(e).lower():
            raise CohereAPIError(f"Cohere API error: {str(e)}")
        else:
            raise e