"""
Unit tests for RAG Chatbot retrieval functionality.

This module provides unit tests for individual functions in the retrieval system
to ensure each component works correctly in isolation.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from retrieval.models import RetrievedChunk, SearchQuery, ValidationResult
from retrieval.search import semantic_search
from retrieval.validation import validate_embeddings, validate_metadata
from retrieval.utils import (
    validate_similarity_score,
    filter_by_similarity_threshold,
    sort_chunks_by_similarity,
    normalize_scores,
    calculate_score_quality_metrics
)


class TestModels(unittest.TestCase):
    """Test the data models"""

    def test_retrieved_chunk_validation(self):
        """Test RetrievedChunk validation"""
        # Valid chunk should work
        chunk = RetrievedChunk(
            chunk_id="test_id",
            text_content="test content",
            similarity_score=0.8,
            source_url="https://example.com",
            title="Test Title",
            section="Test Section",
            created_at="2023-01-01",
            char_start=0,
            char_end=10
        )
        self.assertEqual(chunk.chunk_id, "test_id")

        # Invalid similarity score should raise error
        with self.assertRaises(ValueError):
            RetrievedChunk(
                chunk_id="test_id",
                text_content="test content",
                similarity_score=1.5,  # Invalid: > 1
                source_url="https://example.com",
                title="Test Title",
                section="Test Section",
                created_at="2023-01-01",
                char_start=0,
                char_end=10
            )

        # Empty text should raise error
        with self.assertRaises(ValueError):
            RetrievedChunk(
                chunk_id="test_id",
                text_content="",  # Invalid: empty
                similarity_score=0.8,
                source_url="https://example.com",
                title="Test Title",
                section="Test Section",
                created_at="2023-01-01",
                char_start=0,
                char_end=10
            )

    def test_search_query_validation(self):
        """Test SearchQuery validation"""
        # Valid query should work
        query = SearchQuery("test query", top_k=5, min_score=0.3)
        self.assertEqual(query.query_text, "test query")

        # Invalid query text should raise error
        with self.assertRaises(ValueError):
            SearchQuery("", top_k=5, min_score=0.3)  # Empty query

        # Invalid top_k should raise error
        with self.assertRaises(ValueError):
            SearchQuery("test", top_k=15, min_score=0.3)  # > 10

        # Invalid min_score should raise error
        with self.assertRaises(ValueError):
            SearchQuery("test", top_k=5, min_score=1.5)  # > 1

    def test_validation_result_validation(self):
        """Test ValidationResult validation"""
        # Valid result should work
        result = ValidationResult(
            is_valid=True,
            errors=[],
            warnings=[],
            metrics={"test": 1}
        )
        self.assertTrue(result.is_valid)

        # Invalid result should raise error
        with self.assertRaises(ValueError):
            ValidationResult(
                is_valid=True,
                errors=["some error"],  # Can't have errors with is_valid=True
                warnings=[],
                metrics={"test": 1}
            )


class TestUtils(unittest.TestCase):
    """Test utility functions"""

    def test_validate_similarity_score(self):
        """Test similarity score validation"""
        # Valid scores
        self.assertTrue(validate_similarity_score(0.5))
        self.assertTrue(validate_similarity_score(0.0))
        self.assertTrue(validate_similarity_score(1.0))

        # Invalid scores
        self.assertFalse(validate_similarity_score(-0.1))
        self.assertFalse(validate_similarity_score(1.1))

        # With threshold
        self.assertTrue(validate_similarity_score(0.8, min_threshold=0.5))
        self.assertFalse(validate_similarity_score(0.3, min_threshold=0.5))

    def test_filter_by_similarity_threshold(self):
        """Test filtering chunks by similarity threshold"""
        chunks = [
            RetrievedChunk("1", "content1", 0.9, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("2", "content2", 0.3, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("3", "content3", 0.7, "url", "title", "section", "2023", 0, 10),
        ]

        filtered = filter_by_similarity_threshold(chunks, min_score=0.5)
        self.assertEqual(len(filtered), 2)
        self.assertEqual(filtered[0].chunk_id, "1")

    def test_sort_chunks_by_similarity(self):
        """Test sorting chunks by similarity"""
        chunks = [
            RetrievedChunk("1", "content1", 0.5, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("2", "content2", 0.9, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("3", "content3", 0.7, "url", "title", "section", "2023", 0, 10),
        ]

        sorted_chunks = sort_chunks_by_similarity(chunks)
        self.assertEqual(sorted_chunks[0].similarity_score, 0.9)
        self.assertEqual(sorted_chunks[1].similarity_score, 0.7)
        self.assertEqual(sorted_chunks[2].similarity_score, 0.5)

    def test_normalize_scores(self):
        """Test score normalization"""
        scores = [1, 2, 3, 4, 5]
        normalized = normalize_scores(scores)
        self.assertEqual(normalized[0], 0.0)  # Min becomes 0
        self.assertEqual(normalized[-1], 1.0)  # Max becomes 1

        # Empty list
        self.assertEqual(normalize_scores([]), [])

        # Same values
        same_scores = [5, 5, 5]
        normalized_same = normalize_scores(same_scores)
        self.assertEqual(normalized_same, [1.0, 1.0, 1.0])

    def test_calculate_score_quality_metrics(self):
        """Test score quality metrics calculation"""
        chunks = [
            RetrievedChunk("1", "content1", 0.5, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("2", "content2", 0.9, "url", "title", "section", "2023", 0, 10),
            RetrievedChunk("3", "content3", 0.7, "url", "title", "section", "2023", 0, 10),
        ]

        metrics = calculate_score_quality_metrics(chunks)
        self.assertAlmostEqual(metrics['mean_score'], 0.7, places=7)
        self.assertEqual(metrics['min_score'], 0.5)
        self.assertEqual(metrics['max_score'], 0.9)
        self.assertIsNotNone(metrics['std_deviation'])  # Should be calculated


class TestValidation(unittest.TestCase):
    """Test validation functions"""

    @patch('retrieval.validation.get_qdrant_client')
    def test_validate_embeddings(self, mock_get_client):
        """Test validate_embeddings function"""
        # Mock Qdrant client
        mock_client = Mock()
        mock_get_client.return_value = mock_client

        # Mock collection info
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 1024
        mock_client.get_collection.return_value = mock_collection_info

        # Mock count result
        mock_count_result = Mock()
        mock_count_result.count = 0
        mock_client.count.return_value = mock_count_result

        # Test with no points in collection
        result = validate_embeddings()
        self.assertFalse(result.is_valid)
        self.assertIn("No points found in the collection", result.errors)

    @patch('retrieval.validation.get_qdrant_client')
    def test_validate_metadata(self, mock_get_client):
        """Test validate_metadata function"""
        # Mock Qdrant client
        mock_client = Mock()
        mock_get_client.return_value = mock_client

        # Mock collection info
        mock_collection_info = Mock()
        mock_client.get_collection.return_value = mock_collection_info

        # Mock count result
        mock_count_result = Mock()
        mock_count_result.count = 0
        mock_client.count.return_value = mock_count_result

        # Test with no points in collection
        result = validate_metadata()
        self.assertFalse(result.is_valid)
        self.assertIn("No points found in the collection", result.errors)


class TestSearch(unittest.TestCase):
    """Test search functions"""

    @patch('retrieval.search.get_qdrant_client')
    @patch('retrieval.search.get_cohere_client')
    def test_semantic_search(self, mock_get_cohere, mock_get_qdrant):
        """Test semantic_search function with mocks"""
        # Mock clients
        mock_qdrant = Mock()
        mock_cohere = Mock()
        mock_get_qdrant.return_value = mock_qdrant
        mock_get_cohere.return_value = mock_cohere

        # Mock Cohere embedding response
        mock_embedding_response = Mock()
        mock_embedding_response.embeddings = [[0.1] * 1024]  # 1024-dimensional vector
        mock_cohere.embed.return_value = mock_embedding_response

        # Mock Qdrant search response
        from qdrant_client.http.models import QueryResponse
        mock_search_result = QueryResponse(points=[])
        mock_qdrant.query_points.return_value = mock_search_result

        # Test with valid query
        query = SearchQuery("test query", top_k=3, min_score=0.1)
        result = semantic_search(query)

        # Should return empty list since we mocked empty search results
        self.assertEqual(result, [])

        # Verify the calls were made
        mock_cohere.embed.assert_called_once()
        mock_qdrant.query_points.assert_called_once()


if __name__ == '__main__':
    unittest.main()