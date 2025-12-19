# RAG Ingestion Pipeline

This project implements a Retrieval-Augmented Generation (RAG) ingestion pipeline that extracts text from the Physical AI & Humanoid Robotics book website, generates embeddings using Cohere, and stores them in a Qdrant vector database.

## Overview

The RAG Ingestion Pipeline performs the following steps:
1. **Text Extraction**: Extracts text content from the Physical AI & Humanoid Robotics book website
2. **Text Chunking**: Splits the extracted text into appropriately sized chunks
3. **Embedding Generation**: Creates vector embeddings using Cohere's API
4. **Vector Storage**: Stores the embeddings in Qdrant vector database with metadata

## Prerequisites

- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Clone the repository and navigate to the backend directory**
   ```bash
   cd backend
   ```

2. **Create a virtual environment and activate it**
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   uv pip install -r requirements.txt
   ```

4. **Set up environment variables**
   Copy the `.env` template and add your API keys:
   ```bash
   cp .env .env.local
   ```
   Edit `.env.local` and add your Cohere and Qdrant API keys.

## Configuration

The pipeline can be configured through environment variables in your `.env` file:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: Your Qdrant cluster endpoint URL
- `QDRANT_API_KEY`: Your Qdrant API key for authentication
- `BOOK_BASE_URL`: Base URL for the book website (default: `https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/`)
- `CHUNK_SIZE`: Size of text chunks for embedding (default: 512)
- `CHUNK_OVERLAP`: Overlap between adjacent chunks (default: 50)
- `RATE_LIMIT_DELAY`: Delay in seconds between web requests (default: 1)
- `BATCH_SIZE`: Number of chunks to process per Cohere API call (default: 10)

## Usage

### Run the Full Pipeline

To run the complete RAG ingestion pipeline:

```bash
python main.py
```

### Run Individual Components for Testing

The pipeline includes test functions that can be run individually:

```python
from main import test_text_extraction, test_embedding_generation, test_embedding_storage

# Test text extraction
test_text_extraction()

# Test embedding generation
test_embedding_generation()

# Test embedding storage
test_embedding_storage()
```

## Project Structure

```
backend/
├── main.py                 # Main pipeline implementation
├── constants.py            # Configuration constants
├── utils.py                # Helper functions
├── exceptions.py           # Custom exception classes
├── pyproject.toml          # Project dependencies
├── requirements.txt        # Dependency list
├── .env                    # Environment variables template
├── .gitignore              # Git ignore patterns
└── README.md               # This file
```

## Functions Overview

- `get_all_urls(base_url)`: Discovers all URLs from the book website
- `extract_text_from_url(url)`: Extracts clean text content from a URL
- `chunk_text(text, chunk_size, overlap)`: Splits text into chunks with overlap
- `embed(text_chunks)`: Generates embeddings for text chunks using Cohere
- `create_collection(collection_name, vector_size)`: Creates Qdrant collection
- `save_chunk_to_qdrant(chunk_id, embedding, metadata)`: Stores embeddings in Qdrant
- `main()`: Orchestrates the entire pipeline

## Error Handling

The pipeline includes comprehensive error handling:
- Network request retries with exponential backoff
- Validation of embeddings and metadata
- Graceful degradation when individual URLs fail
- Detailed logging for debugging

## Output

The pipeline generates:
- A log file (`rag_pipeline.log`) with detailed execution information
- Summary statistics showing success rates and processing metrics
- Vector embeddings stored in Qdrant with rich metadata for retrieval

## Testing

The pipeline includes test functions for each major component:
- Text extraction from sample URLs
- Embedding generation with sample text
- Storage of sample embeddings to Qdrant
- End-to-end pipeline functionality

## Performance Considerations

- The pipeline processes URLs in batches to respect rate limits
- Embeddings are generated in batches to optimize API usage
- Exponential backoff is used for retry logic
- Progress tracking is provided via tqdm for user feedback