"""
Shared data models for the RAG Chatbot retrieval system.

This module contains the core data classes used across the retrieval system.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class RetrievedChunk:
    """Represents a text chunk returned by semantic search"""
    chunk_id: str
    text_content: str
    similarity_score: float
    source_url: str
    title: str
    section: str
    created_at: str
    char_start: int
    char_end: int

    def __post_init__(self):
        """Validate the RetrievedChunk after initialization"""
        if not 0 <= self.similarity_score <= 1:
            raise ValueError(f"similarity_score must be between 0 and 1, got {self.similarity_score}")
        if not self.text_content:
            raise ValueError("text_content must not be empty")
        if self.char_start >= self.char_end:
            raise ValueError(f"char_start ({self.char_start}) must be less than char_end ({self.char_end})")


@dataclass
class SearchQuery:
    """Represents a user query for semantic search"""
    query_text: str
    top_k: int = 3
    min_score: float = 0.0

    def __post_init__(self):
        """Validate the SearchQuery after initialization"""
        if not self.query_text:
            raise ValueError("query_text must not be empty")
        if not 1 <= self.top_k <= 10:
            raise ValueError(f"top_k must be between 1 and 10, got {self.top_k}")
        if not 0 <= self.min_score <= 1:
            raise ValueError(f"min_score must be between 0 and 1, got {self.min_score}")


@dataclass
class ValidationResult:
    """Represents the outcome of validation checks"""
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    metrics: dict

    def __post_init__(self):
        """Validate the ValidationResult after initialization"""
        if self.is_valid and self.errors:
            raise ValueError("is_valid cannot be True when errors exist")
        if not isinstance(self.errors, list):
            raise ValueError("errors must be a list")
        if not isinstance(self.warnings, list):
            raise ValueError("warnings must be a list")
        if not isinstance(self.metrics, dict):
            raise ValueError("metrics must be a dict")