"""
Test scripts for RAG Chatbot retrieval functionality.

This module provides functions to test and validate the retrieval system
with sample queries and accuracy measurements.
"""
import logging
from typing import List, Dict
from config.settings import get_qdrant_client, QDRANT_COLLECTION_NAME
from retrieval.search import semantic_search
from retrieval.models import RetrievedChunk, SearchQuery, ValidationResult
from retrieval.utils import calculate_score_quality_metrics


def test_retrieval_accuracy() -> Dict:
    """
    Test retrieval accuracy with predefined sample queries.

    Returns:
        Dict: Results of the accuracy test including metrics
    """
    import time

    # Define sample queries to test (10+ queries as required)
    sample_queries = [
        SearchQuery("robotic nervous system", top_k=5, min_score=0.1),
        SearchQuery("digital twin technology", top_k=5, min_score=0.1),
        SearchQuery("vision-language-action models", top_k=5, min_score=0.1),
        SearchQuery("ROS 2 framework", top_k=5, min_score=0.1),
        SearchQuery("humanoid robot control", top_k=5, min_score=0.1),
        SearchQuery("machine learning in robotics", top_k=5, min_score=0.1),
        SearchQuery("computer vision for robots", top_k=5, min_score=0.1),
        SearchQuery("robotic manipulation", top_k=5, min_score=0.1),
        SearchQuery("autonomous navigation", top_k=5, min_score=0.1),
        SearchQuery("sensor fusion in robotics", top_k=5, min_score=0.1),
        SearchQuery("robotic path planning", top_k=5, min_score=0.1),
        SearchQuery("human-robot interaction", top_k=5, min_score=0.1)
    ]

    results = {
        "total_queries": len(sample_queries),
        "successful_queries": 0,
        "failed_queries": 0,
        "average_chunks_per_query": 0,
        "average_similarity_score": 0.0,
        "relevance_score": 0.0,
        "total_response_time": 0.0,
        "average_response_time": 0.0,
        "queries_results": []
    }

    total_chunks = 0
    total_score = 0.0
    total_relevance_score = 0.0
    total_response_time = 0.0

    for query in sample_queries:
        try:
            logging.info(f"Testing query: {query.query_text}")
            start_time = time.time()
            retrieved_chunks = semantic_search(query)
            end_time = time.time()
            response_time = end_time - start_time
            total_response_time += response_time

            # Calculate metrics for this query
            query_metrics = {
                "query": query.query_text,
                "num_chunks": len(retrieved_chunks),
                "response_time": response_time,
                "chunks": []
            }

            # Calculate relevance score for this query based on similarity scores
            query_relevance_score = 0.0
            if retrieved_chunks:
                # Weighted relevance score based on position and similarity
                for i, chunk in enumerate(retrieved_chunks):
                    # Higher weight for higher-ranked items (position discount)
                    position_weight = 1.0 / (i + 1)
                    query_relevance_score += chunk.similarity_score * position_weight
                    query_metrics["chunks"].append({
                        "chunk_id": chunk.chunk_id,
                        "text_preview": chunk.text_content[:100] + "..." if len(chunk.text_content) > 100 else chunk.text_content,
                        "similarity_score": chunk.similarity_score,
                        "source_url": chunk.source_url
                    })
                    total_score += chunk.similarity_score

            query_metrics["relevance_score"] = query_relevance_score
            total_relevance_score += query_relevance_score

            results["queries_results"].append(query_metrics)
            total_chunks += len(retrieved_chunks)
            results["successful_queries"] += 1

        except Exception as e:
            logging.error(f"Failed to execute query '{query.query_text}': {str(e)}")
            results["failed_queries"] += 1

    # Calculate aggregate metrics
    if results["successful_queries"] > 0:
        results["average_chunks_per_query"] = total_chunks / results["successful_queries"]
        if total_chunks > 0:
            results["average_similarity_score"] = total_score / total_chunks
        if len(sample_queries) > 0:
            results["relevance_score"] = total_relevance_score / len(sample_queries)
        results["total_response_time"] = total_response_time
        results["average_response_time"] = total_response_time / results["successful_queries"]

    logging.info(f"Retrieval accuracy test completed: {results['successful_queries']} successful, {results['failed_queries']} failed")
    logging.info(f"Average relevance score: {results['relevance_score']:.3f}")
    logging.info(f"Average response time: {results['average_response_time']:.3f} seconds")

    return results


def run_sample_queries():
    """
    Execute a set of sample queries to validate retrieval functionality.
    """
    # Test the specific "robotic nervous system" query mentioned in the requirements
    logging.info("Testing semantic search with sample query: 'robotic nervous system'")

    try:
        query = SearchQuery("robotic nervous system", top_k=3, min_score=0.1)
        results = semantic_search(query)

        logging.info(f"Found {len(results)} relevant chunks for query: '{query.query_text}'")

        for i, chunk in enumerate(results, 1):
            logging.info(f"Result {i}: Score={chunk.similarity_score:.3f}, Source={chunk.source_url}")
            logging.info(f"  Content preview: {chunk.text_content[:200]}...")

        # Calculate and log quality metrics
        metrics = calculate_score_quality_metrics(results)
        logging.info(f"Score quality metrics: {metrics}")

        return results

    except Exception as e:
        logging.error(f"Error running sample query: {str(e)}")
        raise


def test_embedding_validation():
    """
    Test embedding validation functionality.
    """
    from retrieval.validation import validate_embeddings

    logging.info("Testing embedding validation...")
    result = validate_embeddings()

    logging.info(f"Embedding validation result: is_valid={result.is_valid}")
    logging.info(f"Metrics: {result.metrics}")

    if result.errors:
        logging.error(f"Validation errors: {result.errors}")

    if result.warnings:
        logging.warning(f"Validation warnings: {result.warnings}")

    return result


def test_metadata_validation():
    """
    Test metadata validation functionality.
    """
    from retrieval.validation import validate_metadata

    logging.info("Testing metadata validation...")
    result = validate_metadata()

    logging.info(f"Metadata validation result: is_valid={result.is_valid}")
    logging.info(f"Metrics: {result.metrics}")

    if result.errors:
        logging.error(f"Validation errors: {result.errors}")

    if result.warnings:
        logging.warning(f"Validation warnings: {result.warnings}")

    return result


def validate_environment():
    """
    Validate that the environment is properly configured for retrieval operations.

    Returns:
        bool: True if environment is properly configured, False otherwise
    """
    import os
    required_env_vars = ["QDRANT_URL", "QDRANT_API_KEY", "COHERE_API_KEY", "QDRANT_COLLECTION_NAME"]

    missing_vars = []
    for var in required_env_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        logging.error(f"Missing required environment variables: {missing_vars}")
        return False

    # Test Qdrant connection
    try:
        from config.settings import get_qdrant_client
        client = get_qdrant_client()
        # Try to connect and get collections
        client.get_collections()
        logging.info("Qdrant connection successful")
    except Exception as e:
        logging.error(f"Qdrant connection failed: {str(e)}")
        return False

    # Test Cohere connection
    try:
        from config.settings import get_cohere_client
        client = get_cohere_client()
        # Try to get embeddings for a simple test
        response = client.embed(
            texts=["test"],
            model="embed-multilingual-v3.0",
            input_type="search_query"
        )
        logging.info("Cohere connection successful")
    except Exception as e:
        logging.error(f"Cohere connection failed: {str(e)}")
        return False

    return True


def validate_data_integrity(accuracy_results, embedding_results, metadata_results):
    """
    Validate data integrity between different retrieval steps.

    Args:
        accuracy_results: Results from retrieval accuracy tests
        embedding_results: Results from embedding validation
        metadata_results: Results from metadata validation

    Returns:
        bool: True if data integrity checks pass, False otherwise
    """
    integrity_issues = []

    # Check if retrieval was successful when embeddings and metadata are valid
    if embedding_results.is_valid and metadata_results.is_valid:
        if accuracy_results and accuracy_results['successful_queries'] == 0:
            integrity_issues.append("Retrieval failed despite valid embeddings and metadata")

    # Check if retrieval accuracy metrics are reasonable
    if accuracy_results:
        if accuracy_results.get('average_similarity_score', 0) < 0.1:
            integrity_issues.append(f"Very low average similarity score: {accuracy_results.get('average_similarity_score', 0):.3f}")

        if accuracy_results.get('relevance_score', 0) < 0.1:
            integrity_issues.append(f"Very low relevance score: {accuracy_results.get('relevance_score', 0):.3f}")

    # Check if validation results are consistent
    if not embedding_results.is_valid and not metadata_results.is_valid:
        integrity_issues.append("Both embedding and metadata validation failed, which may indicate a data storage issue")

    if integrity_issues:
        for issue in integrity_issues:
            logging.warning(f"Data integrity issue: {issue}")
        return False

    logging.info("Data integrity validation passed")
    return True


def main():
    """
    Main function to run comprehensive validation of the retrieval system.
    """
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting comprehensive retrieval system validation...")

    # Validate environment first
    logging.info("Validating environment configuration...")
    if not validate_environment():
        logging.error("Environment validation failed. Please check your configuration.")
        return {
            "retrieval_accuracy": None,
            "embedding_validation": None,
            "metadata_validation": None,
            "error": "Environment validation failed"
        }

    # Run retrieval accuracy tests
    logging.info("Running retrieval accuracy tests...")
    accuracy_results = test_retrieval_accuracy()

    # Run embedding validation
    logging.info("Running embedding validation...")
    embedding_results = test_embedding_validation()

    # Run metadata validation
    logging.info("Running metadata validation...")
    metadata_results = test_metadata_validation()

    # Validate data integrity between steps
    logging.info("Validating data integrity between retrieval steps...")
    integrity_valid = validate_data_integrity(accuracy_results, embedding_results, metadata_results)

    # Summary report
    logging.info("=== VALIDATION SUMMARY ===")
    logging.info(f"Retrieval accuracy tests: {accuracy_results['successful_queries']}/{accuracy_results['total_queries']} successful")
    logging.info(f"Average relevance score: {accuracy_results['relevance_score']:.3f}")
    logging.info(f"Average response time: {accuracy_results['average_response_time']:.3f} seconds")
    logging.info(f"Embedding validation: {'PASSED' if embedding_results.is_valid else 'FAILED'}")
    logging.info(f"Metadata validation: {'PASSED' if metadata_results.is_valid else 'FAILED'}")
    logging.info(f"Data integrity validation: {'PASSED' if integrity_valid else 'FAILED'}")

    # Return comprehensive results
    return {
        "retrieval_accuracy": accuracy_results,
        "embedding_validation": embedding_results,
        "metadata_validation": metadata_results,
        "data_integrity_valid": integrity_valid
    }


if __name__ == "__main__":
    main()