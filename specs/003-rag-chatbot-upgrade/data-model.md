# Data Model: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Created**: 2025-12-16

## Entity: Text Content

### Attributes
- **id** (string, required): Unique identifier for each text chunk, generated as UUID
- **url** (string, required): Source URL where the content was extracted from
- **content** (string, required): The actual text content (cleaned and processed)
- **title** (string, optional): Title or heading associated with the content
- **section** (string, optional): Book section/chapter identifier
- **created_at** (datetime, required): Timestamp of extraction in ISO 8601 format
- **processed** (boolean, required): Flag indicating if content has been embedded (default: false)

### Validation Rules
- content must be non-empty after cleaning
- url must be a valid URL format
- created_at must be in the past or present
- id must be unique across all text content entities

### Relationships
- One-to-many with Embedding Vector (via id → chunk_id)

## Entity: Embedding Vector

### Attributes
- **id** (string, required): Unique identifier matching the source text chunk
- **vector** (array of floats, required): Numerical embedding vector from Cohere (1024-dimensional)
- **metadata** (object, required): Dictionary containing source URL, title, section
- **created_at** (datetime, required): Timestamp of embedding generation in ISO 8601 format
- **collection** (string, required): Name of Qdrant collection where stored (default: "rag_embeddings")

### Validation Rules
- vector must have exactly 1024 elements
- vector elements must be finite numbers (not NaN or Infinity)
- id must match an existing Text Content entity
- metadata must contain required fields (url, title, section)

### Relationships
- Many-to-one with Text Content (via id → Text Content.id)
- Stored in Qdrant vector database with associated metadata

## Entity: Chunk Metadata

### Attributes
- **chunk_id** (string, required): ID of the text chunk, matches Text Content id
- **source_url** (string, required): Original URL of the content
- **position** (integer, required): Sequential position of chunk within original document (0-indexed)
- **char_start** (integer, required): Character offset where chunk starts in original text
- **char_end** (integer, required): Character offset where chunk ends in original text
- **token_count** (integer, required): Number of tokens in the chunk
- **parent_chunk_id** (string, optional): ID of parent chunk if this is a sub-chunk

### Validation Rules
- char_start must be less than char_end
- position must be non-negative
- token_count must be positive and within embedding service limits
- chunk_id must match an existing Text Content entity

### Relationships
- One-to-one with Text Content (via chunk_id → Text Content.id)

## Entity: Processing Job

### Attributes
- **job_id** (string, required): Unique identifier for the processing job
- **status** (string, required): Current status (pending, in_progress, completed, failed)
- **start_time** (datetime, required): When the job started in ISO 8601 format
- **end_time** (datetime, optional): When the job completed/failed in ISO 8601 format
- **total_urls** (integer, required): Total number of URLs to process
- **processed_urls** (integer, required): Number of URLs successfully processed
- **failed_urls** (integer, required): Number of URLs that failed to process
- **total_chunks** (integer, required): Total number of text chunks created
- **embedded_chunks** (integer, required): Number of chunks successfully embedded
- **error_details** (array, optional): List of error messages for failed operations

### Validation Rules
- status must be one of the allowed values
- start_time must be before end_time (if end_time is present)
- processed_urls must not exceed total_urls
- embedded_chunks must not exceed total_chunks

### Relationships
- Contains many Text Content entities processed during the job
- Tracks the progress of the ingestion pipeline

## Qdrant Collection Schema

### Collection Name: "rag_embeddings"

### Vector Configuration
- **Size**: 1024 (dimension of Cohere embeddings)
- **Distance**: Cosine (for semantic similarity)

### Payload Schema
```
{
  "chunk_id": "string",           // Matches Text Content id
  "source_url": "string",         // Original URL
  "title": "string",              // Content title
  "section": "string",            // Book section/chapter
  "position": "integer",          // Position in original document
  "char_start": "integer",        // Start character offset
  "char_end": "integer",          // End character offset
  "token_count": "integer",       // Number of tokens in chunk
  "created_at": "string"          // ISO 8601 timestamp
}
```

### Indexing Strategy
- Index on "source_url" for efficient URL-based queries
- Index on "section" for efficient section-based queries
- Index on "created_at" for time-based queries
- Full-text search index on content for keyword queries

## State Transitions

### Text Content Entity
```
Initial State: Created during URL extraction
  ↓ (embedding process)
State: Embedded (processed = true)
  ↓ (Qdrant storage)
State: Stored in vector database
```

### Processing Job Entity
```
Initial State: pending
  ↓ (start processing)
State: in_progress
  ↓ (completion or failure)
State: completed | failed
```

## Constraints and Business Rules

1. **Data Integrity**: Each embedded chunk must have a corresponding Text Content record
2. **Processing Order**: Text must be chunked before embedding generation
3. **Storage Validation**: Embeddings must be validated before storage in Qdrant
4. **Error Recovery**: Failed chunks should be retryable without reprocessing successful ones
5. **Metadata Consistency**: All metadata fields must be preserved through the pipeline