# Quickstart: RAG Chatbot Step 2 - Retrieval & Pipeline Validation

## Setup

1. **Prerequisites**:
   - Python 3.9+
   - Existing embeddings stored in Qdrant (from Spec-1)

2. **Environment Configuration**:
   ```bash
   cd backend
   # Ensure .env file has correct QDRANT_URL and QDRANT_API_KEY
   ```

3. **Install Dependencies** (if not already installed):
   ```bash
   pip install qdrant-client python-dotenv
   ```

## Basic Retrieval

1. **Run a simple retrieval test**:
   ```python
   from retrieval.search import semantic_search

   # Search for content related to "robotic nervous system"
   results = semantic_search("What is the robotic nervous system?", top_k=3)

   for chunk in results:
       print(f"Score: {chunk.similarity_score}")
       print(f"Content: {chunk.text_content[:200]}...")
       print(f"Source: {chunk.source_url}")
       print("---")
   ```

2. **Validate embeddings**:
   ```python
   from retrieval.validation import validate_embeddings

   # Check that embeddings are properly stored
   validation_result = validate_embeddings()
   print(f"Validation passed: {validation_result.is_valid}")
   print(f"Errors: {validation_result.errors}")
   ```

## Testing Sample Queries

1. **Run validation tests**:
   ```bash
   python -m retrieval.test_retrieval
   ```

2. **Test specific queries**:
   ```python
   from retrieval.test_retrieval import test_retrieval_accuracy

   # Test predefined sample queries
   test_results = test_retrieval_accuracy()
   print(f"Accuracy: {test_results['accuracy']:.2%}")
   ```

## Expected Output

When running retrieval tests, you should see:
- Top-k relevant chunks returned with similarity scores
- Source URLs matching the expected book sections
- Validation results confirming embedding integrity
- Performance metrics showing response times