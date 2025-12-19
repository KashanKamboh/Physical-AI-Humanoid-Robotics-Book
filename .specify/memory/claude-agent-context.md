# Physical AI & Humanoid Robotics Book - Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-16

## Active Technologies

- Python 3.9+
- UV package manager
- Cohere API (embed-multilingual-v3.0 model)
- Qdrant vector database (Cloud)
- BeautifulSoup4 (HTML parsing)
- Requests (HTTP client)
- python-dotenv (environment management)
- tqdm (progress tracking)
- Web scraping techniques
- Text chunking algorithms
- Vector embeddings (1024-dimensional)
- Semantic similarity search

## Project Structure

```text
backend/
├── main.py
├── pyproject.toml
├── requirements.txt
├── .env
└── .gitignore
specs/
└── 003-rag-chatbot-upgrade/
    ├── spec.md
    ├── plan.md
    ├── research.md
    ├── data-model.md
    ├── quickstart.md
    └── contracts/
        └── api-contracts.md
```

## Commands

- `uv venv` - Create virtual environment
- `source .venv/bin/activate` - Activate virtual environment (Linux/Mac)
- `.venv\Scripts\activate` - Activate virtual environment (Windows)
- `uv pip install -r requirements.txt` - Install dependencies
- `python main.py` - Run the RAG ingestion pipeline

## Code Style

- Use type hints for all function parameters and return values
- Follow PEP 8 style guidelines
- Write docstrings for all functions using Google style
- Use meaningful variable names that reflect the domain (e.g., embedding_vector, text_chunk)
- Implement comprehensive error handling for external service calls
- Use environment variables for configuration and API keys
- Structure code in modular functions with single responsibilities

## Recent Changes

- RAG Ingestion Pipeline: Added backend infrastructure with Cohere embeddings and Qdrant storage
- Text Extraction: Implemented web scraping functionality to extract content from book website
- Vector Database: Set up Qdrant collection for semantic search capability

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->