# RAG Chatbot Retrieval Module

This module provides semantic search functionality for the RAG Chatbot, allowing retrieval of relevant text chunks from the Physical AI & Humanoid Robotics book using vector similarity search.

## Overview

The retrieval module implements a semantic search system that:
- Uses Cohere's embedding model to generate query embeddings
- Performs vector similarity search in Qdrant database
- Returns semantically relevant text chunks with metadata
- Includes comprehensive validation and error handling

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Query    │───▶│  semantic_search │───▶│ Qdrant Vector   │
│   (SearchQuery) │    │   (retrieval)    │    │   Database      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │  RetrievedChunk  │
                    │   (Results)      │
                    └──────────────────┘
```

## Installation

1. Ensure you have the required dependencies installed:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables (see Configuration section below)

## Configuration

Create a `.env` file in the project root with the following variables:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=rag_embeddings
COHERE_API_KEY=your_cohere_api_key
```

## Usage

### Basic Semantic Search

```python
from retrieval.models import SearchQuery
from retrieval.search import semantic_search

# Create a search query
query = SearchQuery(
    query_text="robotic nervous system",
    top_k=5,        # Number of results to return (1-10)
    min_score=0.1   # Minimum similarity score threshold (0-1)
)

# Perform semantic search
results = semantic_search(query)

# Process results
for chunk in results:
    print(f"Score: {chunk.similarity_score}")
    print(f"Content: {chunk.text_content[:100]}...")
    print(f"Source: {chunk.source_url}")
    print("---")
```

### Validation

Validate the retrieval system:

```python
from retrieval.validation import validate_embeddings, validate_metadata

# Validate embeddings
embed_result = validate_embeddings()
print(f"Embeddings valid: {embed_result.is_valid}")

# Validate metadata
meta_result = validate_metadata()
print(f"Metadata valid: {meta_result.is_valid}")
```

### Comprehensive Testing

Run the full validation suite:

```python
from retrieval.test_retrieval import main

# Run comprehensive validation
results = main()
print(results)
```

## Data Models

### SearchQuery
- `query_text`: The search query string
- `top_k`: Number of results to return (default: 3, range: 1-10)
- `min_score`: Minimum similarity score (default: 0.0, range: 0-1)

### RetrievedChunk
- `chunk_id`: Unique identifier for the text chunk
- `text_content`: The actual text content
- `similarity_score`: Cosine similarity score (0-1)
- `source_url`: URL where the content originated
- `title`: Title of the source document
- `section`: Section name in the document
- `created_at`: Creation timestamp
- `char_start`: Start character position in source
- `char_end`: End character position in source

### ValidationResult
- `is_valid`: Whether validation passed
- `errors`: List of validation errors
- `warnings`: List of validation warnings
- `metrics`: Dictionary of validation metrics

## API Functions

### semantic_search(query: SearchQuery) -> List[RetrievedChunk]
Performs semantic search and returns relevant text chunks sorted by similarity.

### validate_embeddings() -> ValidationResult
Validates that stored embeddings have correct dimensions and properties.

### validate_metadata() -> ValidationResult
Validates that metadata fields are correctly stored and retrievable.

### test_retrieval_accuracy() -> Dict
Tests retrieval accuracy with predefined sample queries and returns metrics.

## Performance Optimizations

- **Caching**: Query results are cached in memory to improve response times for repeated queries
- **Efficient Filtering**: Results are filtered efficiently based on similarity thresholds
- **Connection Reuse**: Qdrant and Cohere clients are reused across requests

## Error Handling

The module includes comprehensive error handling:
- `QueryValidationError`: Invalid query parameters
- `QdrantConnectionError`: Connection issues with Qdrant
- `CohereAPIError`: Issues with Cohere API
- `EmbeddingDimensionError`: Embedding dimension mismatch
- `MetadataValidationError`: Metadata validation failure
- `RetrievalTimeoutError`: Retrieval operation timeout

## Testing

### Unit Tests
Run unit tests for individual functions:
```bash
python -m pytest backend/retrieval/test_retrieval_unit.py
```

### Integration Tests
Run integration tests for the full pipeline:
```bash
python -m pytest backend/retrieval/test_retrieval_integration.py
```

## Environment Validation

The system validates configuration at startup. Ensure all required environment variables are set before running any retrieval operations.

## Performance Metrics

The system tracks various performance metrics:
- Response time for queries
- Average similarity scores
- Relevance scores
- Number of chunks retrieved per query

## Troubleshooting

1. **Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
2. **API Errors**: Check COHERE_API_KEY and API usage limits
3. **No Results**: Ensure the Qdrant collection has embedded content
4. **Low Quality Results**: Adjust min_score threshold or check embedding quality

## Security

- API keys are loaded from environment variables
- Input validation prevents injection attacks
- Error messages don't expose sensitive information