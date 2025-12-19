"""
Validation functionality for RAG Chatbot retrieval system.

This module provides functions to validate that embeddings and metadata
are correctly stored in Qdrant and can be retrieved properly.
"""
import logging
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config.settings import get_qdrant_client, QDRANT_COLLECTION_NAME, VECTOR_SIZE
from retrieval.models import RetrievedChunk, SearchQuery, ValidationResult
from retrieval.exceptions import QdrantConnectionError, EmbeddingDimensionError, MetadataValidationError


def validate_embeddings() -> ValidationResult:
    """
    Validate that stored embeddings have the correct dimensions and properties.

    Returns:
        ValidationResult: Result of the validation check
    """
    errors = []
    warnings = []
    metrics = {}

    try:
        qdrant_client = get_qdrant_client()

        # Get collection info to verify it exists and get vector config
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)

        # Check if collection has the expected vector size
        if hasattr(collection_info.config.params, 'vectors'):
            vector_config = collection_info.config.params.vectors
            if hasattr(vector_config, 'size'):
                if vector_config.size != VECTOR_SIZE:
                    errors.append(f"Vector size mismatch: expected {VECTOR_SIZE}, got {vector_config.size}")
            else:
                # For older Qdrant versions, check differently
                if hasattr(vector_config, '__getitem__') and 'size' in vector_config:
                    if vector_config['size'] != VECTOR_SIZE:
                        errors.append(f"Vector size mismatch: expected {VECTOR_SIZE}, got {vector_config['size']}")
                else:
                    errors.append(f"Could not determine vector size from collection config")
        else:
            errors.append(f"Could not access vector configuration for collection {QDRANT_COLLECTION_NAME}")

        # Sample a few points to check embedding dimensions
        # Get count of points in collection
        count_result = qdrant_client.count(
            collection_name=QDRANT_COLLECTION_NAME,
            exact=True
        )
        total_points = count_result.count
        metrics['total_points'] = total_points

        if total_points == 0:
            errors.append("No points found in the collection")
        else:
            # Sample up to 10 points to check dimensions
            sample_size = min(10, total_points)
            sample_ids = list(range(min(sample_size, 100)))  # Using first few IDs as sample

            # If we have more than 100 points, create a more random sample
            if total_points > 100:
                import random
                random.seed(42)  # For reproducible results
                sample_ids = random.sample(range(total_points), sample_size)

            # Actually retrieve points by searching with a dummy query or getting specific points
            # Using scroll to get random points from the collection
            points, next_offset = qdrant_client.scroll(
                collection_name=QDRANT_COLLECTION_NAME,
                limit=sample_size,
                with_payload=False,
                with_vectors=True
            )

            dimension_errors = 0
            for i, point in enumerate(points):
                if point.vector is not None:
                    vector_values = point.vector
                    # Handle different vector formats (dict vs list)
                    if isinstance(vector_values, dict):
                        # For named vectors, check the default vector
                        vector_list = list(vector_values.values())[0] if vector_values else []
                    else:
                        vector_list = vector_values

                    if len(vector_list) != VECTOR_SIZE:
                        dimension_errors += 1
                        errors.append(f"Point {point.id} has incorrect vector dimension: {len(vector_list)} instead of {VECTOR_SIZE}")

            metrics['sampled_points'] = len(points)
            metrics['dimension_errors'] = dimension_errors
            metrics['vector_size_correct'] = dimension_errors == 0

            # Check for NaN or Infinity values in vectors
            invalid_vector_count = 0
            for point in points:
                if point.vector is not None:
                    vector_values = point.vector
                    if isinstance(vector_values, dict):
                        vector_list = list(vector_values.values())[0] if vector_values else []
                    else:
                        vector_list = vector_values

                    for value in vector_list:
                        if not (float('-inf') < value < float('inf')):
                            invalid_vector_count += 1
                            errors.append(f"Point {point.id} contains invalid vector value: {value}")
                            break  # Report first invalid value per vector

            metrics['invalid_vector_count'] = invalid_vector_count

        # Summary
        is_valid = len(errors) == 0
        if is_valid:
            logging.info(f"Embedding validation passed: {total_points} points with correct dimensions")
        else:
            logging.error(f"Embedding validation failed: {len(errors)} errors found")

        return ValidationResult(
            is_valid=is_valid,
            errors=errors,
            warnings=warnings,
            metrics=metrics
        )

    except QdrantConnectionError:
        # Specific handling for Qdrant connection errors
        error_msg = "Qdrant connection failed - retrieval system unavailable"
        logging.error(error_msg)
        warnings.append(error_msg)  # Use warning instead of error for graceful degradation

        return ValidationResult(
            is_valid=False,
            errors=[],
            warnings=warnings,
            metrics=metrics
        )
    except Exception as e:
        logging.error(f"Error during embedding validation: {str(e)}")
        # Check if it's a connection-related error
        if "connection" in str(e).lower() or "timeout" in str(e).lower() or "network" in str(e).lower():
            warnings.append(f"Connection issue during validation: {str(e)}")
            return ValidationResult(
                is_valid=False,
                errors=[],
                warnings=warnings,
                metrics=metrics
            )
        else:
            return ValidationResult(
                is_valid=False,
                errors=[f"Embedding validation failed with error: {str(e)}"],
                warnings=warnings,
                metrics=metrics
            )


def validate_metadata() -> ValidationResult:
    """
    Validate that metadata fields are correctly stored and retrievable.

    Returns:
        ValidationResult: Result of the validation check
    """
    errors = []
    warnings = []
    metrics = {}

    try:
        qdrant_client = get_qdrant_client()

        # Get collection info
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)

        # Sample a few points to check metadata fields
        count_result = qdrant_client.count(
            collection_name=QDRANT_COLLECTION_NAME,
            exact=True
        )
        total_points = count_result.count
        metrics['total_points'] = total_points

        if total_points == 0:
            errors.append("No points found in the collection")
        else:
            # Sample up to 20 points to check metadata
            sample_size = min(20, total_points)

            # Use scroll to get points with payload (metadata)
            points, next_offset = qdrant_client.scroll(
                collection_name=QDRANT_COLLECTION_NAME,
                limit=sample_size,
                with_payload=True,
                with_vectors=False
            )

            # Define required metadata fields
            required_fields = ['source_url', 'title', 'section', 'created_at', 'char_start', 'char_end', 'chunk_id']

            missing_field_errors = 0
            invalid_value_errors = 0

            for point in points:
                payload = point.payload

                # Check for required fields
                for field in required_fields:
                    if field not in payload:
                        missing_field_errors += 1
                        errors.append(f"Point {point.id} missing required metadata field: {field}")
                    else:
                        # Validate field values
                        value = payload[field]
                        if field in ['source_url', 'title', 'section', 'text'] and not isinstance(value, str):
                            invalid_value_errors += 1
                            errors.append(f"Point {point.id} has invalid value type for {field}: expected str, got {type(value)}")
                        elif field in ['char_start', 'char_end'] and not isinstance(value, int):
                            invalid_value_errors += 1
                            errors.append(f"Point {point.id} has invalid value type for {field}: expected int, got {type(value)}")
                        elif field == 'created_at' and not isinstance(value, str):
                            invalid_value_errors += 1
                            errors.append(f"Point {point.id} has invalid value type for {field}: expected str, got {type(value)}")
                        elif field in ['source_url', 'title', 'section'] and not value.strip():
                            invalid_value_errors += 1
                            errors.append(f"Point {point.id} has empty value for required field: {field}")
                        elif field == 'text' and not value:
                            invalid_value_errors += 1
                            errors.append(f"Point {point.id} has empty text content")

            metrics['sampled_points'] = len(points)
            metrics['missing_field_errors'] = missing_field_errors
            metrics['invalid_value_errors'] = invalid_value_errors
            metrics['metadata_fields_present'] = missing_field_errors == 0

            # Check if any points have valid metadata
            if len(points) > 0 and missing_field_errors == 0 and invalid_value_errors == 0:
                logging.info(f"Metadata validation passed: {len(points)} points with complete metadata")
            else:
                logging.error(f"Metadata validation failed: {missing_field_errors} missing fields, {invalid_value_errors} invalid values")

        # Summary
        is_valid = len(errors) == 0
        return ValidationResult(
            is_valid=is_valid,
            errors=errors,
            warnings=warnings,
            metrics=metrics
        )

    except QdrantConnectionError:
        # Specific handling for Qdrant connection errors
        error_msg = "Qdrant connection failed - retrieval system unavailable"
        logging.error(error_msg)
        warnings.append(error_msg)  # Use warning instead of error for graceful degradation

        return ValidationResult(
            is_valid=False,
            errors=[],
            warnings=warnings,
            metrics=metrics
        )
    except Exception as e:
        logging.error(f"Error during metadata validation: {str(e)}")
        # Check if it's a connection-related error
        if "connection" in str(e).lower() or "timeout" in str(e).lower() or "network" in str(e).lower():
            warnings.append(f"Connection issue during validation: {str(e)}")
            return ValidationResult(
                is_valid=False,
                errors=[],
                warnings=warnings,
                metrics=metrics
            )
        else:
            return ValidationResult(
                is_valid=False,
                errors=[f"Metadata validation failed with error: {str(e)}"],
                warnings=warnings,
                metrics=metrics
            )