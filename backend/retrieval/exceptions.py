"""
Custom exception classes for the RAG Chatbot retrieval system.

This module defines specific exception types for different error scenarios
in the retrieval process.
"""


class RetrievalError(Exception):
    """Base exception for retrieval-related errors"""
    pass


class QdrantConnectionError(RetrievalError):
    """Raised when there's a connection issue with Qdrant"""
    pass


class CohereAPIError(RetrievalError):
    """Raised when there's an issue with the Cohere API"""
    pass


class QueryValidationError(RetrievalError):
    """Raised when a search query fails validation"""
    pass


class EmbeddingDimensionError(RetrievalError):
    """Raised when embedding dimensions don't match expected values"""
    pass


class MetadataValidationError(RetrievalError):
    """Raised when metadata validation fails"""
    pass


class RetrievalTimeoutError(RetrievalError):
    """Raised when a retrieval operation times out"""
    pass