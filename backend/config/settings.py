"""
Configuration settings for the retrieval system.

This module handles environment variable loading and provides access to
configuration values needed for Qdrant and Cohere clients.
"""
import os
from typing import Optional
from qdrant_client import QdrantClient
import cohere
from dotenv import load_dotenv
import logging


# Load environment variables
load_dotenv()


def validate_config() -> bool:
    """
    Validate that all required configuration values are present and valid.

    Returns:
        bool: True if configuration is valid, False otherwise
    """
    required_vars = ["QDRANT_URL", "QDRANT_API_KEY", "COHERE_API_KEY"]
    missing_vars = []

    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        logging.error(f"Missing required environment variables: {missing_vars}")
        return False

    # Validate QDRANT_COLLECTION_NAME exists or will use default
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings")
    if not collection_name.strip():
        logging.error("QDRANT_COLLECTION_NAME cannot be empty")
        return False

    return True


def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance.

    Returns:
        QdrantClient: Configured Qdrant client
    """
    if not validate_config():
        raise ValueError("Configuration validation failed - missing required environment variables")

    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")

    if not url or not api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

    try:
        return QdrantClient(
            url=url,
            api_key=api_key,
        )
    except Exception as e:
        logging.error(f"Failed to create Qdrant client: {str(e)}")
        raise


def get_cohere_client() -> cohere.Client:
    """
    Create and return a Cohere client instance.

    Returns:
        cohere.Client: Configured Cohere client
    """
    if not validate_config():
        raise ValueError("Configuration validation failed - missing required environment variables")

    api_key = os.getenv("COHERE_API_KEY")

    if not api_key:
        raise ValueError("COHERE_API_KEY must be set in environment variables")

    try:
        return cohere.Client(api_key=api_key)
    except Exception as e:
        logging.error(f"Failed to create Cohere client: {str(e)}")
        raise


# Constants
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings")
VECTOR_SIZE = 1024  # Cohere embed-multilingual-v3.0 returns 1024-dimensional vectors

# Validate configuration on module load
if not validate_config():
    logging.warning("Configuration validation failed - some required environment variables may be missing")