"""
Initialization module for the RAG Chatbot retrieval package.

This module exposes key functions and classes from the retrieval submodules
to make them easily accessible when importing the package.
"""

from .models import RetrievedChunk, SearchQuery, ValidationResult
from .exceptions import (
    RetrievalError,
    QdrantConnectionError,
    CohereAPIError,
    QueryValidationError,
    EmbeddingDimensionError,
    MetadataValidationError,
    RetrievalTimeoutError
)
from .search import semantic_search
from .validation import validate_embeddings, validate_metadata
from .test_retrieval import test_retrieval_accuracy, run_sample_queries
from .utils import (
    validate_similarity_score,
    filter_by_similarity_threshold,
    sort_chunks_by_similarity,
    normalize_scores,
    calculate_score_quality_metrics
)
from .chunking import SemanticChunker, TextChunk, validate_ingestion
from .ingestion import BookIngestor, quick_validation_test

__all__ = [
    # Models
    'RetrievedChunk',
    'SearchQuery',
    'ValidationResult',

    # Exceptions
    'RetrievalError',
    'QdrantConnectionError',
    'CohereAPIError',
    'QueryValidationError',
    'EmbeddingDimensionError',
    'MetadataValidationError',
    'RetrievalTimeoutError',

    # Search functions
    'semantic_search',

    # Validation functions
    'validate_embeddings',
    'validate_metadata',

    # Test functions
    'test_retrieval_accuracy',
    'run_sample_queries',

    # Utility functions
    'validate_similarity_score',
    'filter_by_similarity_threshold',
    'sort_chunks_by_similarity',
    'normalize_scores',
    'calculate_score_quality_metrics',

    # Chunking functions
    'SemanticChunker',
    'TextChunk',
    'validate_ingestion',

    # Ingestion functions
    'BookIngestor',
    'quick_validation_test'
]