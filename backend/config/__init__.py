"""
Initialization module for the config package.

This module exposes key functions and classes from the config submodules
to make them easily accessible when importing the package.
"""

from .settings import get_qdrant_client, get_cohere_client, QDRANT_COLLECTION_NAME, VECTOR_SIZE

__all__ = [
    'get_qdrant_client',
    'get_cohere_client',
    'QDRANT_COLLECTION_NAME',
    'VECTOR_SIZE'
]