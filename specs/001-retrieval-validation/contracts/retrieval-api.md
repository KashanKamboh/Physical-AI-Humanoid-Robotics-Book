# Internal API Contract: Retrieval Functions

## Search Interface

### `semantic_search(query: SearchQuery) -> List[RetrievedChunk]`

**Purpose**: Perform semantic search on embedded book content

**Input**:
- `query_text`: User's search query (string)
- `top_k`: Number of results to return (integer, default: 3)
- `min_score`: Minimum similarity threshold (float, default: 0.0)

**Output**:
- List of `RetrievedChunk` objects sorted by similarity score (descending)

**Error Conditions**:
- Invalid query parameters
- Qdrant connection failure
- No embeddings found in database

### `validate_embeddings() -> ValidationResult`

**Purpose**: Validate that embeddings are correctly stored in Qdrant

**Input**: None

**Output**:
- `ValidationResult` object with validation status and metrics

**Error Conditions**:
- Qdrant connection failure
- Invalid embedding dimensions
- Missing metadata fields

### `test_retrieval_accuracy() -> Dict`

**Purpose**: Run predefined test queries to validate retrieval accuracy

**Input**: None

**Output**:
- Dictionary with accuracy metrics and test results

**Error Conditions**:
- Test query execution failure
- Validation failure