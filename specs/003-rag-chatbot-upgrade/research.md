# Research Document: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Created**: 2025-12-16
**Researcher**: Claude

## Research Task 1: Cohere Embedding Model Selection

### Decision: Use embed-multilingual-v3.0 model
**Rationale**: The Physical AI & Humanoid Robotics book may contain technical terms, code snippets, and potentially content in multiple languages. The multilingual model provides better performance for diverse technical content and maintains compatibility with English-heavy technical documentation.

**Alternatives considered**:
- embed-english-v3.0: More optimized for English but potentially less robust for technical terms
- embed-multilingual-light-v3.0: Faster but potentially less accurate for complex technical content

## Research Task 2: Qdrant Configuration Best Practices

### Decision: Use Cosine distance with 1024-dimensional vectors
**Rationale**: Cohere's multilingual embedding model produces 1024-dimensional vectors. Cosine distance is optimal for semantic similarity in high-dimensional embedding spaces and is the recommended approach for embedding similarity.

**Configuration details**:
- Collection name: "rag_embeddings"
- Vector size: 1024 (matching Cohere embedding dimensions)
- Distance metric: Cosine
- Additional payload: URL, title, section, and timestamp metadata

## Research Task 3: Text Chunking Strategy

### Decision: 512-token chunks with 50-token overlap
**Rationale**: 512 tokens provides sufficient context for semantic understanding while staying within reasonable API limits. The 50-token overlap ensures continuity of meaning across chunk boundaries, which is important for technical content where concepts may span multiple sentences.

**Alternatives considered**:
- 256-token chunks: More granular but potentially loses contextual meaning
- 1024-token chunks: More context but higher API costs and potentially less precision

## Research Task 4: Web Scraping Best Practices

### Decision: Use requests + BeautifulSoup with rate limiting
**Rationale**: For a static documentation site like the book website, requests + BeautifulSoup provides efficient, lightweight scraping without the overhead of a full browser automation tool. Adding rate limiting respects server resources while maintaining reasonable processing speed.

**Implementation approach**:
- Use requests.Session() for connection reuse
- Add 1-second delays between requests to respect rate limits
- Parse HTML with BeautifulSoup using 'html.parser'
- Extract content from main content areas while preserving text structure

## Additional Technical Considerations

### Rate Limits and API Management
- Cohere API has rate limits (typically 100-1000 RPM depending on plan)
- Implement exponential backoff for API retries
- Batch requests to maximize efficiency (up to 96 texts per request)

### Error Handling Strategy
- Network errors: Retry with exponential backoff (5 attempts max)
- API errors: Log and skip problematic chunks, continue processing
- Parsing errors: Validate content before processing, fallback to alternative extraction

### Environment Variables Required
- `COHERE_API_KEY`: Cohere API key for embedding generation
- `QDRANT_URL`: Qdrant cluster endpoint URL
- `QDRANT_API_KEY`: Qdrant API key for authentication
- `BOOK_BASE_URL`: Base URL for the book website (default: https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/)

### Configuration Parameters
- `CHUNK_SIZE`: 512 tokens (default)
- `CHUNK_OVERLAP`: 50 tokens (default)
- `RATE_LIMIT_DELAY`: 1 second between web requests (default)
- `BATCH_SIZE`: 10 (number of chunks to process per Cohere API call)

## Validation and Testing Approach

### Unit Testing
- Individual function testing with mock data
- Validation of embedding dimensions (should be 1024)
- Verification of metadata preservation

### Integration Testing
- End-to-end pipeline testing with sample URLs
- Verification that stored embeddings can be retrieved and searched
- Performance testing with realistic content volumes

### Quality Assurance
- Manual verification of extracted text quality
- Semantic similarity testing of stored embeddings
- Coverage validation (ensure all book sections are processed)