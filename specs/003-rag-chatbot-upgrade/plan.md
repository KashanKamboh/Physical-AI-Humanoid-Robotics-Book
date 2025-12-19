# Implementation Plan: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Created**: 2025-12-16
**Status**: Draft
**Branch**: 003-rag-chatbot-upgrade

## Technical Context

This plan outlines the architecture for the RAG ingestion pipeline that will extract text from the Physical AI & Humanoid Robotics book website, generate embeddings using Cohere, and store them in Qdrant vector database. The implementation will be contained in a single `main.py` file with well-defined functions for each step of the process.

### Knowns:
- Target website: https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/
- Required functionality: text extraction, chunking, embedding generation, vector storage
- Dependencies: Cohere API, Qdrant vector database
- Project structure: backend/ folder with UV package manager

### Unknowns (NEEDS CLARIFICATION):
- Specific Cohere embedding model to use (multilingual vs English-specific)
- Qdrant Cloud configuration details (cluster endpoint, API key management)
- Exact text chunking strategy (size, overlap, splitting method)
- Rate limits for Cohere API calls
- Error handling strategy for failed URL requests
- Specific text cleaning requirements for the book content

## Constitution Check

### Accuracy (from constitution)
- All embedding generation must use authoritative Cohere models
- Text extraction must preserve original content without modification
- Data integrity must be maintained during chunking and storage

### Clarity (from constitution)
- Code must be well-documented with clear function comments
- Each function should have clear input/output specifications
- Error messages must be informative and actionable

### Reproducibility (from constitution)
- All dependencies must be specified in pyproject.toml
- Environment setup must be documented in quickstart.md
- API keys and configuration must be handled via environment variables

### Rigor (from constitution)
- Proper error handling for network requests, API calls, and database operations
- Validation of embedding dimensions and metadata consistency
- Proper handling of different text encodings and formats

### Execution Standards (from constitution)
- Code must be structured in a modular, testable way
- Functions must have clear boundaries and responsibilities
- Final implementation must be deployable and maintainable

## Gates

### Gate 1: Architecture Alignment
✅ PASS - Architecture aligns with feature requirements (extraction → embedding → storage)

### Gate 2: Constitution Compliance
✅ PASS - Plan addresses all constitution principles with specific measures

### Gate 3: Implementation Feasibility
✅ PASS - All required technologies are available and documented

---

## Phase 0: Research & Resolution

### Research Task 1: Cohere Embedding Model Selection
**Objective**: Determine the most appropriate Cohere embedding model for technical book content
- Compare performance of different models (embed-english-v3.0, embed-multilingual-v3.0)
- Consider token limits and cost implications
- Evaluate embedding quality for technical/robotics content

### Research Task 2: Qdrant Configuration Best Practices
**Objective**: Establish optimal Qdrant collection configuration for book content
- Determine appropriate vector dimensions based on Cohere model
- Plan collection schema with proper metadata fields
- Consider performance implications of different distance metrics

### Research Task 3: Text Chunking Strategy
**Objective**: Define optimal text chunking approach for book content
- Evaluate different chunk sizes (256, 512, 1024 tokens)
- Determine overlap strategy to preserve context
- Consider chapter/section boundaries for coherent chunks

### Research Task 4: Web Scraping Best Practices
**Objective**: Identify appropriate approach for extracting content from the book website
- Evaluate different scraping libraries (requests + BeautifulSoup vs scrapy vs Playwright)
- Consider respect for robots.txt and rate limiting
- Plan for handling different content types and structures

---

## Phase 1: Design & Contracts

### Data Model: `data-model.md`

#### Text Content Entity
- **id**: Unique identifier for each text chunk
- **url**: Source URL where the content was extracted from
- **content**: The actual text content (cleaned and processed)
- **title**: Title or heading associated with the content
- **section**: Book section/chapter identifier
- **created_at**: Timestamp of extraction
- **processed**: Boolean flag indicating if content has been embedded

#### Embedding Vector Entity
- **id**: Unique identifier matching the source text chunk
- **vector**: Numerical embedding vector from Cohere
- **metadata**: Dictionary containing source URL, title, section
- **created_at**: Timestamp of embedding generation
- **collection**: Name of Qdrant collection where stored

#### Chunk Metadata Entity
- **chunk_id**: ID of the text chunk
- **source_url**: Original URL of the content
- **position**: Sequential position of chunk within original document
- **char_start**: Character offset where chunk starts in original text
- **char_end**: Character offset where chunk ends in original text
- **token_count**: Number of tokens in the chunk

### API Contracts: `/contracts/`

#### Embedding Generation Contract
- **Input**: Text content (string), chunk size parameters
- **Output**: Embedding vector (array of floats), metadata
- **Error Handling**: Cohere API errors, rate limiting, invalid input
- **Validation**: Text length limits, encoding validation

#### Vector Storage Contract
- **Input**: Embedding vector, metadata, collection name
- **Output**: Storage confirmation, vector ID
- **Error Handling**: Database connection errors, storage limits
- **Validation**: Vector dimension validation, metadata completeness

### Quickstart Guide: `quickstart.md`

```bash
# Setup backend environment
cd backend/
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
uv pip install -r requirements.txt

# Set environment variables
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="your-qdrant-cluster-url"
export QDRANT_API_KEY="your-qdrant-api-key"

# Run the ingestion pipeline
python main.py
```

### System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Source URLs   │───▶│  Text Extraction │───▶│   Text Chunks   │
│ (Book Website)  │    │    & Cleaning    │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Qdrant Vector  │◀───│  Embedding &    │◀───│  Chunk Storage  │
│   Database      │    │   Metadata       │    │   & Metadata    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                      ┌──────────────────┐
                      │   Cohere API     │
                      │  Embedding Gen   │
                      └──────────────────┘
```

### Function Design

#### `get_all_urls(base_url: str) -> List[str]`
- **Purpose**: Discover and return all URLs from the book website
- **Input**: Base URL of the book website
- **Output**: List of all page URLs to extract content from
- **Implementation**: Parse the website structure, follow navigation links
- **Error Handling**: Network errors, invalid URLs, rate limiting

#### `extract_text_from_url(url: str) -> Dict[str, str]`
- **Purpose**: Extract clean text content from a single URL
- **Input**: URL to extract content from
- **Output**: Dictionary with 'title', 'content', and 'section' keys
- **Implementation**: Web scraping with BeautifulSoup, content cleaning
- **Error Handling**: HTTP errors, parsing failures, content validation

#### `chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, str]]`
- **Purpose**: Split text into appropriately sized chunks for embedding
- **Input**: Text content and chunking parameters
- **Output**: List of text chunks with metadata
- **Implementation**: Split by sentences or paragraphs while respecting boundaries
- **Error Handling**: Empty text, invalid chunk sizes

#### `embed(text_chunks: List[str]) -> List[List[float]]`
- **Purpose**: Generate embeddings for text chunks using Cohere API
- **Input**: List of text chunks to embed
- **Output**: List of embedding vectors
- **Implementation**: Batch calls to Cohere embedding API
- **Error Handling**: API rate limits, network errors, invalid text

#### `create_collection(collection_name: str, vector_size: int) -> bool`
- **Purpose**: Initialize a Qdrant collection for storing embeddings
- **Input**: Collection name and expected vector dimensions
- **Output**: Success status
- **Implementation**: Qdrant client call to create collection
- **Error Handling**: Collection already exists, database connection errors

#### `save_chunk_to_qdrant(chunk_id: str, embedding: List[float], metadata: Dict) -> bool`
- **Purpose**: Store a single embedding with metadata in Qdrant
- **Input**: Chunk ID, embedding vector, and associated metadata
- **Output**: Success status
- **Implementation**: Qdrant client call to upsert point
- **Error Handling**: Database errors, invalid vector dimensions

#### `main() -> None`
- **Purpose**: Orchestrate the entire ingestion pipeline
- **Implementation**: Sequential execution of all pipeline steps
- **Error Handling**: Comprehensive error handling and logging
- **Logging**: Progress tracking and success/failure reporting

### Dependencies & Configuration

#### Backend Dependencies (`pyproject.toml`)
- `cohere`: For embedding generation
- `qdrant-client`: For vector database operations
- `beautifulsoup4`: For HTML parsing
- `requests`: For HTTP requests
- `python-dotenv`: For environment variable management
- `tqdm`: For progress tracking

#### Environment Configuration
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_URL`: URL endpoint for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `CHUNK_SIZE`: Size of text chunks for embedding
- `CHUNK_OVERLAP`: Overlap between adjacent chunks

---

## Phase 2: Implementation Strategy

### Development Approach
1. **Setup**: Create backend/ directory with UV package management
2. **Foundation**: Implement core functions with basic functionality
3. **Integration**: Connect functions in main() orchestration
4. **Testing**: Validate each function with sample data
5. **Optimization**: Fine-tune chunking and embedding parameters

### Error Handling Strategy
- Network request retries with exponential backoff
- Graceful degradation when individual URLs fail
- Comprehensive logging for debugging and monitoring
- Validation of embeddings before storage

### Performance Considerations
- Batch processing for efficient Cohere API usage
- Parallel text extraction for multiple URLs
- Memory management for large text chunks
- Connection pooling for Qdrant operations

## Re-evaluation: Post-Design Constitution Check

### Accuracy Verification
✅ All technical decisions based on authoritative sources (Cohere docs, Qdrant docs)
✅ Embedding generation uses proven models from reputable provider
✅ Data integrity maintained through the pipeline

### Clarity Verification
✅ Function boundaries clearly defined with specific responsibilities
✅ Error messages will be informative and actionable
✅ Code structure supports maintainability

### Reproducibility Verification
✅ All dependencies documented in pyproject.toml
✅ Environment setup documented in quickstart guide
✅ Configuration managed via environment variables

### Rigor Verification
✅ Proper error handling for all external service calls
✅ Validation of data formats and dimensions
✅ Robust handling of different text encodings and structures

### Execution Standards Verification
✅ Modular code structure enables testing and maintenance
✅ Clear documentation for onboarding new developers
✅ Production-ready error handling and logging