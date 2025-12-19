# API Contracts: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Created**: 2025-12-16

## Contract: Embedding Generation

### Function Signature
```python
def embed(text_chunks: List[str]) -> List[List[float]]
```

### Input Contract
- **Parameter**: `text_chunks` - List of text strings to generate embeddings for
- **Type**: `List[str]` - Non-empty list of strings
- **Constraints**:
  - Each string must be non-empty after whitespace trimming
  - Each string must be within Cohere API character limits (max ~4000 chars per chunk)
  - Total list length should not exceed Cohere batch limits (96 items per request)

### Output Contract
- **Return Type**: `List[List[float]]` - List of embedding vectors
- **Structure**: Each embedding vector is a list of 1024 floating-point numbers
- **Constraints**:
  - Length of returned list equals length of input list
  - Each embedding vector has exactly 1024 elements
  - All values are finite numbers (no NaN or Infinity)

### Error Contract
- **CohereAPIError**: Raised when Cohere API is unavailable or returns error
- **ValueError**: Raised when input validation fails
- **RateLimitError**: Raised when API rate limits are exceeded

### Example Usage
```python
# Input
text_chunks = ["This is the first text chunk", "This is the second text chunk"]

# Output
embeddings = [
    [0.12, -0.45, 0.89, ...],  # 1024-dimensional vector for first chunk
    [-0.23, 0.67, -0.34, ...]  # 1024-dimensional vector for second chunk
]
```

---

## Contract: Vector Storage

### Function Signature
```python
def save_chunk_to_qdrant(chunk_id: str, embedding: List[float], metadata: Dict) -> bool
```

### Input Contract
- **Parameter**: `chunk_id` - Unique identifier for the text chunk
- **Type**: `str` - Non-empty string
- **Parameter**: `embedding` - The embedding vector to store
- **Type**: `List[float]` - 1024-dimensional vector
- **Parameter**: `metadata` - Additional information to store with the vector
- **Type**: `Dict` - Dictionary containing at least 'source_url', 'title', 'section' keys

### Output Contract
- **Return Type**: `bool` - Success status of the storage operation
- **True**: Vector successfully stored in Qdrant
- **False**: Storage operation failed

### Error Contract
- **QdrantError**: Raised when Qdrant database is unavailable
- **ValidationError**: Raised when embedding dimensions or metadata are invalid
- **ConnectionError**: Raised when unable to connect to Qdrant

### Example Usage
```python
# Input
chunk_id = "abc123-def456-ghi789"
embedding = [0.12, -0.45, 0.89, ...]  # 1024-dimensional vector
metadata = {
    "source_url": "https://example.com/book/chapter1",
    "title": "Introduction to Robotics",
    "section": "Chapter 1",
    "position": 0,
    "char_start": 0,
    "char_end": 512,
    "token_count": 128
}

# Output
success = True  # Indicates successful storage
```

---

## Contract: Text Extraction

### Function Signature
```python
def extract_text_from_url(url: str) -> Dict[str, str]
```

### Input Contract
- **Parameter**: `url` - The URL to extract text content from
- **Type**: `str` - Valid URL string
- **Constraints**: Must be accessible and return HTML content

### Output Contract
- **Return Type**: `Dict[str, str]` - Dictionary with content information
- **Required Keys**:
  - `"title"`: The title of the page
  - `"content"`: The extracted text content (cleaned)
  - `"section"`: The book section identifier
- **Constraints**: All values must be non-empty strings

### Error Contract
- **RequestError**: Raised when URL is inaccessible or returns non-200 status
- **ParseError**: Raised when HTML parsing fails
- **ContentError**: Raised when no content is found at the URL

### Example Usage
```python
# Input
url = "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/chapter1"

# Output
result = {
    "title": "Chapter 1: Introduction to Physical AI",
    "content": "Physical AI represents a convergence of artificial intelligence and physical systems...",
    "section": "Chapter 1"
}
```

---

## Contract: Text Chunking

### Function Signature
```python
def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, str]]
```

### Input Contract
- **Parameter**: `text` - The text to be chunked
- **Type**: `str` - Non-empty string
- **Parameter**: `chunk_size` - Target size of each chunk in tokens
- **Type**: `int` - Positive integer (default: 512)
- **Parameter**: `overlap` - Number of tokens to overlap between chunks
- **Type**: `int` - Non-negative integer (default: 50)

### Output Contract
- **Return Type**: `List[Dict[str, str]]` - List of chunk dictionaries
- **Each Dictionary Contains**:
  - `"content"`: The text content of the chunk
  - `"char_start"`: Starting character position in original text
  - `"char_end"`: Ending character position in original text
  - `"position"`: Sequential position of chunk (0-indexed)
- **Constraints**: Content should preserve sentence boundaries where possible

### Error Contract
- **ValueError**: Raised when chunk_size is not positive or overlap is negative
- **ContentError**: Raised when text is empty or invalid

### Example Usage
```python
# Input
text = "This is a long text that will be split into multiple chunks..."
chunk_size = 512
overlap = 50

# Output
chunks = [
    {
        "content": "This is the first chunk of text...",
        "char_start": 0,
        "char_end": 512,
        "position": 0
    },
    {
        "content": "This is the second chunk overlapping with previous...",
        "char_start": 462,
        "char_end": 978,
        "position": 1
    }
]
```

---

## Contract: Collection Creation

### Function Signature
```python
def create_collection(collection_name: str, vector_size: int) -> bool
```

### Input Contract
- **Parameter**: `collection_name` - Name for the Qdrant collection
- **Type**: `str` - Valid collection name
- **Parameter**: `vector_size` - Dimension of vectors to be stored
- **Type**: `int` - Positive integer (should be 1024 for Cohere embeddings)

### Output Contract
- **Return Type**: `bool` - Success status of collection creation
- **True**: Collection successfully created or already exists
- **False**: Collection creation failed

### Error Contract
- **QdrantError**: Raised when Qdrant database is unavailable
- **ValidationError**: Raised when vector_size is invalid
- **PermissionError**: Raised when insufficient permissions to create collection

### Example Usage
```python
# Input
collection_name = "rag_embeddings"
vector_size = 1024

# Output
success = True  # Indicates successful collection creation
```

---

## Integration Contract: Main Pipeline

### Function Signature
```python
def main() -> None
```

### Process Contract
1. **Initialization**: Validate environment variables and API connectivity
2. **URL Discovery**: Retrieve all book website URLs using `get_all_urls()`
3. **Content Extraction**: Process each URL using `extract_text_from_url()`
4. **Text Chunking**: Split content using `chunk_text()`
5. **Embedding Generation**: Create embeddings using `embed()`
6. **Vector Storage**: Store embeddings using `save_chunk_to_qdrant()`
7. **Error Handling**: Log and continue on individual failures
8. **Reporting**: Output processing summary and statistics

### Success Criteria
- All accessible URLs are processed
- All extracted text is chunked and embedded
- All embeddings are stored in Qdrant
- Processing summary is logged with success/failure counts

### Failure Criteria
- Environment validation fails
- Critical API connectivity issues
- Unable to create Qdrant collection