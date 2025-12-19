# Quickstart Guide: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Created**: 2025-12-16

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Access to Cohere API (API key required)
- Access to Qdrant Cloud (cluster URL and API key required)

## Setup Instructions

### 1. Create Backend Directory
```bash
mkdir backend
cd backend
```

### 2. Initialize UV Environment
```bash
# Create virtual environment
uv venv

# Activate virtual environment
# On Linux/Mac:
source .venv/bin/activate
# On Windows:
.venv\Scripts\activate
```

### 3. Create Project with UV
```bash
# Initialize a new Python project
uv init

# Or if starting from scratch:
uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv tqdm
```

### 4. Set Up Environment Variables
Create a `.env` file in the backend directory:
```bash
# .env
COHERE_API_KEY="your-cohere-api-key-here"
QDRANT_URL="your-qdrant-cluster-url-here"
QDRANT_API_KEY="your-qdrant-api-key-here"
BOOK_BASE_URL="https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/"
CHUNK_SIZE=512
CHUNK_OVERLAP=50
RATE_LIMIT_DELAY=1
BATCH_SIZE=10
```

### 5. Install Dependencies
```bash
# Create requirements.txt from installed packages
pip freeze > requirements.txt
```

## Configuration Parameters

| Variable | Default | Description |
|----------|---------|-------------|
| `COHERE_API_KEY` | *required* | Your Cohere API key for embedding generation |
| `QDRANT_URL` | *required* | Qdrant cluster endpoint URL |
| `QDRANT_API_KEY` | *required* | Qdrant API key for authentication |
| `BOOK_BASE_URL` | `https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/` | Base URL of the book website |
| `CHUNK_SIZE` | 512 | Target size of text chunks in tokens |
| `CHUNK_OVERLAP` | 50 | Number of tokens to overlap between chunks |
| `RATE_LIMIT_DELAY` | 1 | Delay in seconds between web requests |
| `BATCH_SIZE` | 10 | Number of chunks to process per Cohere API call |

## Running the Pipeline

### 1. Create the Main Script
Create a `main.py` file with the following structure:
```python
import os
import requests
import cohere
from qdrant_client import QdrantClient
from bs4 import BeautifulSoup
from dotenv import load_dotenv
import time
from typing import List, Dict, Tuple
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

def get_all_urls(base_url: str) -> List[str]:
    """Discover and return all URLs from the book website."""
    # Implementation here
    pass

def extract_text_from_url(url: str) -> Dict[str, str]:
    """Extract clean text content from a single URL."""
    # Implementation here
    pass

def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, str]]:
    """Split text into appropriately sized chunks for embedding."""
    # Implementation here
    pass

def embed(text_chunks: List[str]) -> List[List[float]]:
    """Generate embeddings for text chunks using Cohere API."""
    # Implementation here
    pass

def create_collection(collection_name: str, vector_size: int) -> bool:
    """Initialize a Qdrant collection for storing embeddings."""
    # Implementation here
    pass

def save_chunk_to_qdrant(chunk_id: str, embedding: List[float], metadata: Dict) -> bool:
    """Store a single embedding with metadata in Qdrant."""
    # Implementation here
    pass

def main():
    """Orchestrate the entire ingestion pipeline."""
    # Implementation here
    pass

if __name__ == "__main__":
    main()
```

### 2. Execute the Pipeline
```bash
# Make sure your environment is activated
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Run the ingestion pipeline
python main.py
```

## Expected Output

The pipeline will output progress information as it processes:
```
Starting RAG ingestion pipeline...
Discovered 25 URLs from the book website
Processing URL 1/25: https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/intro
  - Extracted 1,248 tokens of text content
  - Created 3 text chunks
Processing URL 2/25: https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/chapter1
  - Extracted 2,156 tokens of text content
  - Created 5 text chunks
...
Embedding generation completed: 47 chunks processed
Vector storage completed: 47 embeddings stored in Qdrant
Pipeline completed successfully!
  - Total URLs processed: 25
  - Total text chunks: 47
  - Total embeddings stored: 47
  - Processing time: 345 seconds
```

## Troubleshooting

### Common Issues

**Issue**: "CohereAPIError: Invalid API key"
**Solution**: Verify your COHERE_API_KEY in the .env file is correct and active

**Issue**: "QdrantConnectionError: Cannot connect to Qdrant"
**Solution**: Check that QDRANT_URL and QDRANT_API_KEY are correct in .env file

**Issue**: "RateLimitError: Too many requests"
**Solution**: Increase RATE_LIMIT_DELAY in environment variables

**Issue**: "No content found at URL"
**Solution**: Verify the BOOK_BASE_URL is accessible and contains expected content

### Verification Steps

1. **Test API Connectivity**:
```python
import cohere
co = cohere.Client(os.getenv("COHERE_API_KEY"))
response = co.embed(texts=["test"], model="embed-multilingual-v3.0")
print(f"Embedding test successful: {len(response.embeddings[0])} dimensions")
```

2. **Test Qdrant Connection**:
```python
from qdrant_client import QdrantClient
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
collections = client.get_collections()
print(f"Qdrant connection successful: {len(collections.collections)} collections found")
```

## Next Steps

After successful execution:
1. Verify embeddings are stored in Qdrant
2. Test similarity search functionality
3. Plan the next phase of RAG implementation
4. Set up monitoring for the ingestion pipeline