# Data Model: RAG Chatbot Step 2 - Retrieval & Pipeline Validation

## Entities

### RetrievedChunk
Represents a text chunk returned by semantic search

**Fields**:
- `chunk_id`: Unique identifier for the chunk (string)
- `text_content`: The actual text content of the chunk (string)
- `similarity_score`: Cosine similarity score between query and chunk (float)
- `source_url`: URL where the original content was found (string)
- `title`: Title of the page/chapter containing the chunk (string)
- `section`: Section name within the book (string)
- `created_at`: Timestamp when the chunk was embedded (string, ISO 8601 format)
- `char_start`: Character offset where this chunk starts in the original text (int)
- `char_end`: Character offset where this chunk ends in the original text (int)

**Validation Rules**:
- `similarity_score` must be between 0 and 1
- `chunk_id` must be a valid UUID format
- `text_content` must not be empty
- `source_url` must be a valid URL format
- `char_start` must be less than `char_end`

### SearchQuery
Represents a user query for semantic search

**Fields**:
- `query_text`: The actual text of the user's query (string)
- `top_k`: Number of top results to retrieve (int, default: 3)
- `min_score`: Minimum similarity score threshold (float, default: 0.0)

**Validation Rules**:
- `query_text` must not be empty
- `top_k` must be between 1 and 10
- `min_score` must be between 0 and 1

### ValidationResult
Represents the outcome of validation checks

**Fields**:
- `is_valid`: Whether the validation passed (boolean)
- `errors`: List of error messages if validation failed (array of strings)
- `warnings`: List of warnings during validation (array of strings)
- `metrics`: Dictionary of validation metrics (object)

**Validation Rules**:
- `is_valid` is false if any errors exist
- `errors` and `warnings` must be arrays