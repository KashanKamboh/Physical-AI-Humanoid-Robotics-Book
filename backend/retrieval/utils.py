"""
Utility functions for the RAG Chatbot retrieval system.

This module provides helper functions for similarity score validation
and other common operations in the retrieval pipeline.
"""

from typing import List
from retrieval.models import RetrievedChunk


def validate_similarity_score(score: float, min_threshold: float = 0.0) -> bool:
    """
    Validate that a similarity score is within acceptable bounds and meets threshold.

    Args:
        score: Similarity score to validate (should be between 0 and 1)
        min_threshold: Minimum acceptable similarity score (default 0.0)

    Returns:
        bool: True if score is valid and meets threshold, False otherwise
    """
    if not 0 <= score <= 1:
        return False
    if score < min_threshold:
        return False
    return True


def filter_by_similarity_threshold(chunks: List[RetrievedChunk], min_score: float = 0.0) -> List[RetrievedChunk]:
    """
    Filter retrieved chunks based on a minimum similarity score threshold.

    Args:
        chunks: List of RetrievedChunk objects to filter
        min_score: Minimum similarity score threshold (default 0.0)

    Returns:
        List of RetrievedChunk objects with scores meeting the threshold
    """
    return [chunk for chunk in chunks if validate_similarity_score(chunk.similarity_score, min_score)]


def sort_chunks_by_similarity(chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
    """
    Sort retrieved chunks by similarity score in descending order.

    Args:
        chunks: List of RetrievedChunk objects to sort

    Returns:
        List of RetrievedChunk objects sorted by similarity score (highest first)
    """
    return sorted(chunks, key=lambda chunk: chunk.similarity_score, reverse=True)


def normalize_scores(scores: List[float]) -> List[float]:
    """
    Normalize a list of scores to the range [0, 1].

    Args:
        scores: List of scores to normalize

    Returns:
        List of normalized scores in the range [0, 1]
    """
    if not scores:
        return []

    min_score = min(scores)
    max_score = max(scores)

    # If all scores are the same, return 1.0 for all
    if min_score == max_score:
        return [1.0] * len(scores)

    # Normalize to [0, 1] range
    range_size = max_score - min_score
    return [(score - min_score) / range_size for score in scores]


def calculate_score_quality_metrics(chunks: List[RetrievedChunk]) -> dict:
    """
    Calculate quality metrics for similarity scores of retrieved chunks.

    Args:
        chunks: List of RetrievedChunk objects

    Returns:
        Dictionary containing score quality metrics
    """
    if not chunks:
        return {
            'mean_score': 0.0,
            'median_score': 0.0,
            'min_score': 0.0,
            'max_score': 0.0,
            'std_deviation': 0.0,
            'score_range': 0.0
        }

    scores = [chunk.similarity_score for chunk in chunks]
    mean_score = sum(scores) / len(scores)

    # Calculate median
    sorted_scores = sorted(scores)
    n = len(sorted_scores)
    if n % 2 == 0:
        median_score = (sorted_scores[n // 2 - 1] + sorted_scores[n // 2]) / 2
    else:
        median_score = sorted_scores[n // 2]

    min_score = min(scores)
    max_score = max(scores)
    score_range = max_score - min_score

    # Calculate standard deviation
    variance = sum((score - mean_score) ** 2 for score in scores) / len(scores)
    std_deviation = variance ** 0.5

    return {
        'mean_score': mean_score,
        'median_score': median_score,
        'min_score': min_score,
        'max_score': max_score,
        'std_deviation': std_deviation,
        'score_range': score_range
    }