# Tasks: RAG Ingestion Pipeline

**Feature**: RAG Chatbot Upgrade – Spec 1
**Branch**: 003-rag-chatbot-upgrade
**Created**: 2025-12-16

## Implementation Strategy

Build the RAG ingestion pipeline incrementally, starting with foundational setup and progressing through each user story. The MVP will include basic text extraction functionality that can be tested independently before adding embedding and storage capabilities.

## Dependencies

User stories must be completed in priority order:
- User Story 1 (P1) - Text Content Extraction: Foundation for all other stories
- User Story 2 (P2) - Embedding Generation: Depends on successful text extraction
- User Story 3 (P3) - Embedding Storage: Depends on successful embedding generation

## Parallel Execution Examples

Each user story can be developed in parallel after foundational setup:
- **User Story 1**: Focus on `extract_text_from_url` and `get_all_urls` functions
- **User Story 2**: Focus on `chunk_text` and `embed` functions
- **User Story 3**: Focus on `create_collection` and `save_chunk_to_qdrant` functions

---

## Phase 1: Setup

Initialize the backend project structure with all required dependencies and configuration.

- [x] T001 Create backend/ directory structure
- [x] T002 Initialize Python project with UV package manager in backend/
- [x] T003 [P] Create pyproject.toml with dependencies: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv, tqdm
- [x] T004 [P] Create requirements.txt from pyproject.toml
- [x] T005 Create .env file template with environment variables
- [x] T006 Create main.py file with imports and basic structure
- [x] T007 Create .gitignore for Python project
- [x] T008 Set up logging configuration in main.py

---

## Phase 2: Foundational Components

Implement foundational components that are required by multiple user stories.

- [x] T009 Create constants.py for configuration values (CHUNK_SIZE, CHUNK_OVERLAP, etc.)
- [x] T010 [P] Create utils.py with helper functions for text processing
- [x] T011 [P] Create error handling classes for specific exceptions
- [x] T012 [P] Create environment variable loading and validation function
- [x] T013 [P] Create Cohere client initialization function
- [x] T014 [P] Create Qdrant client initialization function
- [x] T015 Create UUID generation utility for chunk IDs
- [x] T016 [P] Create timestamp utility for ISO 8601 formatting

---

## Phase 3: User Story 1 - Text Content Extraction (Priority: P1)

As a developer working on the RAG chatbot upgrade, I need to extract text content from the Physical AI & Humanoid Robotics book website URLs so that I can create embeddings for the RAG system.

**Independent Test**: Can be fully tested by running the extraction script against the book website URLs and verifying that clean, readable text content is returned without HTML tags or navigation elements.

**Acceptance Scenarios**:
1. Given a valid book website URL, When I run the text extraction process, Then clean text content is extracted without HTML tags, navigation elements, or irrelevant sections
2. Given a book website with multiple pages/chapters, When I run the extraction process, Then all relevant text content from all pages is collected and organized appropriately

- [x] T017 [US1] Implement get_all_urls function to discover all book website URLs
- [x] T018 [US1] Implement extract_text_from_url function using requests and BeautifulSoup
- [x] T019 [US1] Add HTML parsing logic to extract main content areas from book pages
- [x] T020 [US1] Add text cleaning to remove HTML tags, navigation elements, and irrelevant sections
- [x] T021 [US1] Add error handling for network requests and invalid URLs
- [x] T022 [US1] Add rate limiting to respect server resources (1 second delay)
- [x] T023 [US1] Add retry logic with exponential backoff for network requests
- [x] T024 [US1] Add validation to ensure extracted content is non-empty
- [x] T025 [US1] Test text extraction with sample URLs from the book website
- [x] T026 [US1] Add logging for extraction progress and errors

---

## Phase 4: User Story 2 - Embedding Generation (Priority: P2)

As a developer working on the RAG chatbot upgrade, I need to generate embeddings using Cohere so that the text content can be semantically searched and retrieved.

**Independent Test**: Can be fully tested by providing text content to the embedding generation process and verifying that valid vector embeddings are produced that represent the semantic meaning of the text.

**Acceptance Scenarios**:
1. Given extracted text content, When I run the Cohere embedding generation process, Then a valid embedding vector is produced for each text chunk
2. Given multiple text chunks, When I run batch embedding generation, Then all embeddings are successfully generated without errors

- [x] T027 [US2] Implement chunk_text function with 512-token chunks and 50-token overlap
- [x] T028 [US2] Add sentence/paragraph boundary preservation in chunking logic
- [x] T029 [US2] Add character offset tracking (char_start, char_end) in chunk metadata
- [x] T030 [US2] Implement embed function using Cohere API with embed-multilingual-v3.0 model
- [x] T031 [US2] Add batch processing for efficient Cohere API usage (up to 96 items per request)
- [x] T032 [US2] Add validation to ensure embedding vectors have exactly 1024 dimensions
- [x] T033 [US2] Add error handling for Cohere API rate limits and failures
- [x] T034 [US2] Add retry logic with exponential backoff for API calls
- [x] T035 [US2] Add embedding validation to check for NaN or Infinity values
- [x] T036 [US2] Test embedding generation with sample text chunks
- [x] T037 [US2] Add logging for embedding progress and statistics

---

## Phase 5: User Story 3 - Embedding Storage in Vector Database (Priority: P3)

As a developer working on the RAG chatbot upgrade, I need to store the generated embeddings in a vector database so that they can be efficiently retrieved for the RAG system.

**Independent Test**: Can be fully tested by storing embeddings in the vector database and verifying that they can be successfully retrieved and searched against.

**Acceptance Scenarios**:
1. Given generated embeddings, When I store them in the vector database, Then they are successfully indexed and available for search
2. Given stored embeddings in the vector database, When I perform a similarity search, Then relevant embeddings are returned based on semantic similarity

- [x] T038 [US3] Implement create_collection function for Qdrant with 1024-dim vectors and cosine distance
- [x] T039 [US3] Create Qdrant collection schema with proper payload fields (chunk_id, source_url, title, section, etc.)
- [x] T040 [US3] Implement save_chunk_to_qdrant function to store embeddings with metadata
- [x] T041 [US3] Add proper metadata structure matching the data model requirements
- [x] T042 [US3] Add validation for vector dimensions before storage
- [x] T043 [US3] Add error handling for Qdrant connection and storage failures
- [x] T044 [US3] Add retry logic for Qdrant operations
- [x] T045 [US3] Add indexing configuration for efficient queries (source_url, section, created_at)
- [x] T046 [US3] Test embedding storage with sample embeddings
- [x] T047 [US3] Add logging for storage progress and success/failure counts

---

## Phase 6: Main Pipeline Integration

Orchestrate the entire ingestion pipeline in the main() function with comprehensive error handling and logging.

- [x] T048 Implement main() function with pipeline orchestration logic
- [x] T049 Add environment validation at pipeline start
- [x] T050 Add Processing Job entity tracking with status and statistics
- [x] T051 Add comprehensive error handling and logging throughout pipeline
- [x] T052 Add progress tracking with tqdm for user feedback
- [x] T053 Add graceful degradation when individual URLs fail
- [x] T054 Add validation of data integrity between pipeline steps
- [x] T055 Add summary reporting at pipeline completion
- [x] T056 Test full pipeline with sample URLs and verify end-to-end functionality

---

## Phase 7: Polish & Cross-Cutting Concerns

Final touches and quality improvements to ensure the pipeline is production-ready.

- [x] T057 Add comprehensive documentation for all functions
- [x] T058 Add type hints to all functions for better code clarity
- [x] T059 Add unit tests for individual functions
- [x] T060 Add integration tests for the full pipeline
- [x] T061 Optimize performance based on testing results
- [x] T062 Add configuration validation and error messages
- [x] T063 Add data validation for all input and output
- [x] T064 Create README.md with usage instructions
- [x] T065 Run full pipeline on complete book website and verify results