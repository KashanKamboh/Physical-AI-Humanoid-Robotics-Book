"""
RAG Ingestion Pipeline for Physical AI & Humanoid Robotics Book

This script implements the RAG ingestion pipeline that will:
1. Extract text from the Physical AI & Humanoid Robotics book website
2. Generate embeddings using Cohere
3. Store embeddings in Qdrant vector database
"""
import os
import logging
from typing import List, Dict, Tuple, Optional
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import uuid
from datetime import datetime
import json
from tqdm import tqdm
import constants
import utils
from exceptions import (
    CohereAPIError,
    QdrantError,
    ValidationError,
    RequestError,
    ParseError,
    ContentError
)


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('rag_pipeline.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def validate_environment() -> bool:
    """Validate that all required environment variables are set."""
    required_vars = [
        'COHERE_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        return False

    logger.info("All required environment variables are present")
    return True


def get_cohere_client() -> cohere.Client:
    """Create and return a Cohere client instance."""
    api_key = os.getenv('COHERE_API_KEY')
    if not api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    return cohere.Client(api_key=api_key)


def get_qdrant_client() -> QdrantClient:
    """Create and return a Qdrant client instance."""
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')

    if not url or not api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variables not set")

    return QdrantClient(
        url=url,
        api_key=api_key
    )


def get_all_urls(base_url: str) -> List[str]:
    """
    Discover and return all URLs from the book website.

    Args:
        base_url: Base URL of the book website

    Returns:
        List of all page URLs to extract content from

    Raises:
        RequestError: If network request fails
        ParseError: If HTML parsing fails
    """
    try:
        response = requests.get(base_url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Find all links that are relative to the base URL or point to the same domain
        urls = set()
        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)

            # Only include URLs from the same domain
            if urlparse(full_url).netloc == urlparse(base_url).netloc:
                urls.add(full_url)

        # Add the base URL itself
        urls.add(base_url)

        logger.info(f"Discovered {len(urls)} URLs from {base_url}")
        return list(urls)

    except requests.exceptions.RequestException as e:
        logger.error(f"Network error when fetching {base_url}: {str(e)}")
        raise RequestError(f"Failed to fetch URLs from {base_url}: {str(e)}",
                         getattr(e, 'response', None) and getattr(e.response, 'status_code', None))
    except Exception as e:
        logger.error(f"Error parsing HTML from {base_url}: {str(e)}")
        raise ParseError(f"Failed to parse HTML from {base_url}: {str(e)}")


def extract_text_from_url(url: str) -> Dict[str, str]:
    """
    Extract clean text content from a single URL.

    Args:
        url: URL to extract content from

    Returns:
        Dictionary with 'title', 'content', and 'section' keys

    Raises:
        RequestError: If network request fails
        ParseError: If HTML parsing fails
        ContentError: If no content is found
    """
    # Add retry logic with exponential backoff for network requests
    for attempt in range(constants.MAX_RETRIES):
        try:
            # Add rate limiting to respect server resources
            time.sleep(constants.RATE_LIMIT_DELAY)

            response = requests.get(url)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Try to extract the main content area
            # Look for common content containers
            content_selectors = [
                'main',
                '.main-content',
                '.content',
                '.post-content',
                '.article-content',
                '.markdown-section',  # Common in documentation sites
                'article',
                '.container'
            ]

            content_element = None
            for selector in content_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    break

            # If no specific content container found, use body
            if not content_element:
                content_element = soup.find('body')

            if content_element:
                content = content_element.get_text(separator=' ')
            else:
                content = soup.get_text(separator=' ')

            # Clean up the content
            content = utils.clean_text(content)

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else "No Title"

            # Extract section info from URL or heading
            section = urlparse(url).path.strip('/').split('/')[-1] or "Introduction"

            # Validate that we have content
            if not content or len(content.strip()) == 0:
                raise ContentError(f"No content found at {url}")

            logger.info(f"Successfully extracted content from {url} (title: {title[:50]}...)")

            return {
                "title": title,
                "content": content,
                "section": section
            }

        except requests.exceptions.RequestException as e:
            logger.warning(f"Attempt {attempt + 1} failed for {url}: {str(e)}")
            if attempt == constants.MAX_RETRIES - 1:  # Last attempt
                logger.error(f"Network error when fetching {url} after {constants.MAX_RETRIES} attempts: {str(e)}")
                raise RequestError(f"Failed to fetch content from {url} after {constants.MAX_RETRIES} attempts: {str(e)}",
                                 getattr(e, 'response', None) and getattr(e.response, 'status_code', None))
            # Exponential backoff: wait 2^attempt seconds
            time.sleep(2 ** attempt)
        except Exception as e:
            logger.error(f"Error extracting text from {url}: {str(e)}")
            raise ParseError(f"Failed to parse content from {url}: {str(e)}")

    # This line should not be reached due to the exception handling, but included for completeness
    raise RequestError(f"Failed to fetch content from {url} after {constants.MAX_RETRIES} attempts")


def chunk_text(text: str, chunk_size: int = constants.CHUNK_SIZE, overlap: int = constants.CHUNK_OVERLAP) -> List[Dict[str, str]]:
    """
    Split text into appropriately sized chunks for embedding.

    Args:
        text: Text content to be chunked
        chunk_size: Target size of each chunk in tokens (default from constants)
        overlap: Number of tokens to overlap between chunks (default from constants)

    Returns:
        List of text chunks with metadata

    Raises:
        ValidationError: If chunk_size is not positive or overlap is negative
        ContentError: If text is empty or invalid
    """
    if not text or len(text.strip()) == 0:
        raise ContentError("Cannot chunk empty text")

    if chunk_size <= 0:
        raise ValidationError("chunk_size must be positive")

    if overlap < 0:
        raise ValidationError("overlap must be non-negative")

    # For this implementation, we'll use a simple approach based on character count
    # since we don't have access to a proper tokenizer
    text_length = len(text)
    chunks = []

    start_idx = 0
    position = 0

    while start_idx < text_length:
        # Calculate end index
        end_idx = start_idx + chunk_size

        # If we're near the end, adjust to avoid going over
        if end_idx > text_length:
            end_idx = text_length

        # Extract the chunk
        chunk_content = text[start_idx:end_idx]

        # Add to chunks list with metadata
        chunks.append({
            "content": chunk_content,
            "char_start": start_idx,
            "char_end": end_idx,
            "position": position
        })

        # Move to the next chunk with overlap
        start_idx = end_idx - overlap
        position += 1

        # Ensure we don't get stuck in an infinite loop
        if start_idx <= start_idx:  # If overlap >= chunk_size
            break

    logger.info(f"Text chunked into {len(chunks)} chunks")
    return chunks


def embed(text_chunks: List[str]) -> List[List[float]]:
    """
    Generate embeddings for text chunks using Cohere API.

    Args:
        text_chunks: List of text strings to generate embeddings for

    Returns:
        List of embedding vectors (each vector is a list of floats)

    Raises:
        CohereAPIError: If Cohere API returns an error
        ValidationError: If input validation fails
        RateLimitError: If API rate limits are exceeded
    """
    if not text_chunks:
        raise ValidationError("text_chunks cannot be empty")

    # Validate each chunk
    for i, chunk in enumerate(text_chunks):
        if not chunk or len(chunk.strip()) == 0:
            raise ValidationError(f"Text chunk at index {i} is empty or contains only whitespace")

    # Process in batches to respect API limits
    all_embeddings = []

    for i in range(0, len(text_chunks), constants.COHERE_MAX_BATCH_SIZE):
        batch = text_chunks[i:i + constants.COHERE_MAX_BATCH_SIZE]

        # Add retry logic with exponential backoff for API calls
        for attempt in range(constants.MAX_RETRIES):
            try:
                # Get Cohere client
                cohere_client = get_cohere_client()

                # Generate embeddings for the batch
                response = cohere_client.embed(
                    texts=batch,
                    model=constants.COHERE_MODEL,
                    input_type="search_document"
                )

                # Extract embeddings from response
                batch_embeddings = response.embeddings
                all_embeddings.extend(batch_embeddings)

                logger.info(f"Processed batch {i//constants.COHERE_MAX_BATCH_SIZE + 1}, "
                           f"generated {len(batch_embeddings)} embeddings")
                break  # Success, break out of retry loop

            except Exception as e:
                logger.warning(f"Attempt {attempt + 1} failed for embedding batch: {str(e)}")
                if attempt == constants.MAX_RETRIES - 1:  # Last attempt
                    logger.error(f"Failed to generate embeddings after {constants.MAX_RETRIES} attempts: {str(e)}")
                    raise CohereAPIError(f"Failed to generate embeddings: {str(e)}")
                # Exponential backoff: wait 2^attempt seconds
                time.sleep(2 ** attempt)

    # Validate that all embeddings have the correct dimensions
    for i, embedding in enumerate(all_embeddings):
        if len(embedding) != constants.VECTOR_SIZE:
            raise ValidationError(
                f"Embedding at index {i} has incorrect size: {len(embedding)}, "
                f"expected: {constants.VECTOR_SIZE}"
            )

    # Validate that no embedding contains NaN or Infinity values
    import math
    for i, embedding in enumerate(all_embeddings):
        for j, value in enumerate(embedding):
            if not isinstance(value, (int, float)) or math.isnan(value) or math.isinf(value):
                raise ValidationError(
                    f"Embedding at index {i} contains invalid value at position {j}: {value}"
                )

    logger.info(f"Successfully generated embeddings for {len(text_chunks)} text chunks")
    return all_embeddings


def test_embedding_generation():
    """
    Test embedding generation with sample text chunks.
    """
    try:
        sample_text = "This is a sample text chunk for testing embedding generation."
        text_chunks = [sample_text]

        logger.info("Testing embedding generation with sample text")
        embeddings = embed(text_chunks)

        logger.info(f"Generated {len(embeddings)} embeddings")

        if len(embeddings) == 1:
            embedding = embeddings[0]
            if len(embedding) == constants.VECTOR_SIZE:
                logger.info(f"✓ Embedding has correct dimensions: {len(embedding)}")
                logger.info("✓ Embedding generation test passed")
                return True
            else:
                logger.error(f"✗ Embedding has incorrect dimensions: {len(embedding)}, expected: {constants.VECTOR_SIZE}")
                return False
        else:
            logger.error(f"✗ Expected 1 embedding, got {len(embeddings)}")
            return False

    except Exception as e:
        logger.error(f"✗ Embedding generation test failed with error: {str(e)}")
        return False


def test_embedding_generation():
    """
    Test embedding generation with sample text chunks.
    """
    try:
        sample_text = "This is a sample text chunk for testing embedding generation."
        text_chunks = [sample_text]

        logger.info("Testing embedding generation with sample text")
        embeddings = embed(text_chunks)

        logger.info(f"Generated {len(embeddings)} embeddings")

        if len(embeddings) == 1:
            embedding = embeddings[0]
            if len(embedding) == constants.VECTOR_SIZE:
                logger.info(f"✓ Embedding has correct dimensions: {len(embedding)}")
                logger.info("✓ Embedding generation test passed")
                return True
            else:
                logger.error(f"✗ Embedding has incorrect dimensions: {len(embedding)}, expected: {constants.VECTOR_SIZE}")
                return False
        else:
            logger.error(f"✗ Expected 1 embedding, got {len(embeddings)}")
            return False

    except Exception as e:
        logger.error(f"✗ Embedding generation test failed with error: {str(e)}")
        return False


def create_collection(collection_name: str = constants.QDRANT_COLLECTION_NAME, vector_size: int = constants.VECTOR_SIZE) -> bool:
    """
    Initialize a Qdrant collection for storing embeddings.

    Args:
        collection_name: Name for the Qdrant collection (default from constants)
        vector_size: Dimension of vectors to be stored (default from constants)

    Returns:
        Success status of collection creation

    Raises:
        QdrantError: If Qdrant database is unavailable
        ValidationError: If vector_size is invalid
        PermissionError: If insufficient permissions to create collection
    """
    # Add retry logic for Qdrant operations
    for attempt in range(constants.MAX_RETRIES):
        try:
            # Get Qdrant client
            qdrant_client = get_qdrant_client()

            # Check if collection already exists
            try:
                existing_collections = qdrant_client.get_collections()
                collection_exists = any(col.name == collection_name for col in existing_collections.collections)
            except:
                collection_exists = False

            if not collection_exists:
                # Create collection with specified parameters and proper payload schema
                qdrant_client.recreate_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE  # Using cosine distance as specified in requirements
                    ),
                    # Add indexing configuration for efficient queries
                    optimizers_config=models.OptimizersConfigDiff(
                        memmap_threshold=20000,
                        indexing_threshold=20000
                    )
                )

                # Create field indices for efficient queries (source_url, section, created_at)
                qdrant_client.create_payload_index(
                    collection_name=collection_name,
                    field_name="source_url",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                qdrant_client.create_payload_index(
                    collection_name=collection_name,
                    field_name="section",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                qdrant_client.create_payload_index(
                    collection_name=collection_name,
                    field_name="created_at",
                    field_schema=models.PayloadSchemaType.DATETIME
                )

                logger.info(f"Created Qdrant collection '{collection_name}' with {vector_size}-dimensional vectors and indices")
            else:
                logger.info(f"Qdrant collection '{collection_name}' already exists")

            return True

        except Exception as e:
            logger.warning(f"Attempt {attempt + 1} failed to create collection: {str(e)}")
            if attempt == constants.MAX_RETRIES - 1:  # Last attempt
                logger.error(f"Failed to create Qdrant collection after {constants.MAX_RETRIES} attempts: {str(e)}")
                raise QdrantError(f"Failed to create Qdrant collection: {str(e)}")
            # Exponential backoff: wait 2^attempt seconds
            time.sleep(2 ** attempt)


def save_chunk_to_qdrant(chunk_id: str, embedding: List[float], metadata: Dict) -> bool:
    """
    Store a single embedding with metadata in Qdrant.

    Args:
        chunk_id: Unique identifier for the text chunk
        embedding: The embedding vector to store (1024-dimensional)
        metadata: Additional information to store with the vector

    Returns:
        Success status of the storage operation

    Raises:
        QdrantError: If Qdrant database is unavailable
        ValidationError: If embedding dimensions or metadata are invalid
        ConnectionError: If unable to connect to Qdrant
    """
    # Add validation for vector dimensions before storage
    if not chunk_id or len(chunk_id.strip()) == 0:
        raise ValidationError("chunk_id cannot be empty")

    if not embedding or len(embedding) != constants.VECTOR_SIZE:
        raise ValidationError(
            f"Embedding must have exactly {constants.VECTOR_SIZE} dimensions, "
            f"got {len(embedding) if embedding else 0}"
        )

    if not metadata or not isinstance(metadata, dict):
        raise ValidationError("metadata must be a non-empty dictionary")

    # Add proper metadata structure matching the data model requirements
    required_metadata_fields = ['source_url', 'title', 'section']
    for field in required_metadata_fields:
        if field not in metadata:
            raise ValidationError(f"Required metadata field '{field}' is missing")

    # Add retry logic for Qdrant operations
    for attempt in range(constants.MAX_RETRIES):
        try:
            # Get Qdrant client
            qdrant_client = get_qdrant_client()

            # Prepare the point to be stored
            point = models.PointStruct(
                id=chunk_id,
                vector=embedding,
                payload=metadata
            )

            # Store the embedding in Qdrant
            qdrant_client.upsert(
                collection_name=constants.QDRANT_COLLECTION_NAME,
                points=[point]
            )

            logger.info(f"Successfully stored embedding with ID {chunk_id} in Qdrant")
            return True

        except Exception as e:
            logger.warning(f"Attempt {attempt + 1} failed to store embedding: {str(e)}")
            if attempt == constants.MAX_RETRIES - 1:  # Last attempt
                logger.error(f"Failed to store embedding after {constants.MAX_RETRIES} attempts: {str(e)}")
                raise QdrantError(f"Failed to store embedding in Qdrant: {str(e)}")
            # Exponential backoff: wait 2^attempt seconds
            time.sleep(2 ** attempt)


def test_embedding_storage():
    """
    Test embedding storage with sample embeddings.
    """
    try:
        # Create a sample embedding
        sample_embedding = [0.1] * constants.VECTOR_SIZE  # Create a 1024-dim vector
        chunk_id = utils.generate_uuid()

        # Create sample metadata following the data model requirements
        sample_metadata = {
            "chunk_id": chunk_id,
            "source_url": "https://test.example.com/test",
            "title": "Test Title",
            "section": "Test Section",
            "position": 0,
            "char_start": 0,
            "char_end": 100,
            "token_count": 20,
            "created_at": utils.get_iso_timestamp()
        }

        logger.info("Testing embedding storage with sample data")

        # First ensure the collection exists
        create_collection()

        # Store the sample embedding
        success = save_chunk_to_qdrant(chunk_id, sample_embedding, sample_metadata)

        if success:
            logger.info("✓ Embedding storage test passed")
            return True
        else:
            logger.error("✗ Embedding storage test failed - storage operation returned False")
            return False

    except Exception as e:
        logger.error(f"✗ Embedding storage test failed with error: {str(e)}")
        return False


def test_text_extraction():
    """
    Test text extraction with sample URLs from the book website.
    """
    sample_url = os.getenv('BOOK_BASE_URL', constants.DEFAULT_BOOK_BASE_URL)

    try:
        logger.info(f"Testing text extraction from: {sample_url}")
        result = extract_text_from_url(sample_url)

        logger.info(f"Title: {result['title']}")
        logger.info(f"Section: {result['section']}")
        logger.info(f"Content length: {len(result['content'])} characters")

        if len(result['content']) > 0:
            logger.info("✓ Text extraction test passed")
            return True
        else:
            logger.error("✗ Text extraction test failed - no content extracted")
            return False

    except Exception as e:
        logger.error(f"✗ Text extraction test failed with error: {str(e)}")
        return False


def process_url(url: str, progress_bar=None) -> List[Dict]:
    """
    Process a single URL through the full pipeline: extract → chunk → embed → store.

    Args:
        url: URL to process
        progress_bar: Optional tqdm progress bar to update

    Returns:
        List of processed chunk results with metadata
    """
    results = []
    try:
        logger.info(f"Processing URL: {url}")

        # Step 1: Extract text from URL
        text_data = extract_text_from_url(url)
        logger.info(f"Extracted text from {url} (title: {text_data['title'][:50]}...)")

        # Step 2: Chunk the text
        chunks = chunk_text(text_data['content'])
        logger.info(f"Text chunked into {len(chunks)} chunks")

        # Process each chunk
        for i, chunk in enumerate(chunks):
            try:
                # Add metadata to chunk
                chunk_metadata = {
                    "chunk_id": utils.generate_uuid(),
                    "source_url": url,
                    "title": text_data['title'],
                    "section": text_data['section'],
                    "position": chunk.get("position", i),
                    "char_start": chunk.get("char_start", 0),
                    "char_end": chunk.get("char_end", 0),
                    "token_count": utils.count_tokens(chunk["content"]),
                    "created_at": utils.get_iso_timestamp()
                }

                # Step 3: Generate embedding for the chunk
                embedding = embed([chunk["content"]])[0]  # Get first (and only) embedding
                logger.debug(f"Generated embedding for chunk {i} of {url}")

                # Step 4: Store the embedding in Qdrant
                success = save_chunk_to_qdrant(
                    chunk_id=chunk_metadata["chunk_id"],
                    embedding=embedding,
                    metadata=chunk_metadata
                )

                if success:
                    results.append({
                        "chunk_id": chunk_metadata["chunk_id"],
                        "url": url,
                        "success": True,
                        "error": None
                    })
                    logger.debug(f"Successfully stored chunk {i} of {url}")
                else:
                    results.append({
                        "chunk_id": chunk_metadata["chunk_id"],
                        "url": url,
                        "success": False,
                        "error": "Storage operation failed"
                    })
                    logger.error(f"Failed to store chunk {i} of {url}")

            except Exception as e:
                logger.error(f"Error processing chunk {i} of {url}: {str(e)}")
                results.append({
                    "chunk_id": None,
                    "url": url,
                    "success": False,
                    "error": str(e)
                })

        if progress_bar:
            progress_bar.update(1)

    except Exception as e:
        logger.error(f"Error processing URL {url}: {str(e)}")
        results.append({
            "chunk_id": None,
            "url": url,
            "success": False,
            "error": str(e)
        })

    return results


def main():
    """
    Orchestrate the entire ingestion pipeline with comprehensive error handling and logging.
    """
    logger.info("Starting RAG ingestion pipeline...")

    # Step 1: Validate environment
    if not validate_environment():
        logger.error("Environment validation failed. Please check your .env file.")
        return False

    # Step 2: Create Qdrant collection
    logger.info("Creating Qdrant collection...")
    try:
        create_collection()
        logger.info("Qdrant collection created/verified successfully")
    except Exception as e:
        logger.error(f"Failed to create Qdrant collection: {str(e)}")
        return False

    # Step 3: Get all URLs from the book website
    base_url = os.getenv('BOOK_BASE_URL', constants.DEFAULT_BOOK_BASE_URL)
    logger.info(f"Discovering URLs from: {base_url}")
    try:
        urls = get_all_urls(base_url)
        logger.info(f"Discovered {len(urls)} URLs to process")
    except Exception as e:
        logger.error(f"Failed to get URLs from {base_url}: {str(e)}")
        return False

    # Step 4: Process each URL through the pipeline
    total_urls = len(urls)
    processed_urls = 0
    failed_urls = 0
    total_chunks = 0
    embedded_chunks = 0

    logger.info("Starting URL processing...")

    # Create a progress bar
    with tqdm(total=total_urls, desc="Processing URLs", unit="url") as pbar:
        for url in urls:
            try:
                chunk_results = process_url(url, pbar)

                # Update statistics
                processed_urls += 1
                total_chunks += len(chunk_results)
                embedded_chunks += sum(1 for result in chunk_results if result["success"])

                # Count failed URLs
                if not any(result["success"] for result in chunk_results):
                    failed_urls += 1

                # Add some logging for progress
                successful_chunks = sum(1 for result in chunk_results if result["success"])
                logger.debug(f"Completed {url}: {successful_chunks}/{len(chunk_results)} chunks successful")

            except Exception as e:
                logger.error(f"Failed to process {url}: {str(e)}")
                failed_urls += 1
                pbar.update(1)  # Update progress bar even for failed URLs

    # Step 5: Generate summary report
    logger.info("Pipeline completed. Generating summary report...")
    logger.info(f"  Total URLs processed: {total_urls}")
    logger.info(f"  Successfully processed URLs: {processed_urls - failed_urls}")
    logger.info(f"  Failed URLs: {failed_urls}")
    logger.info(f"  Total text chunks created: {total_chunks}")
    logger.info(f"  Successfully embedded and stored chunks: {embedded_chunks}")

    # Step 6: Validation
    success_rate = (processed_urls - failed_urls) / total_urls if total_urls > 0 else 0
    storage_success_rate = embedded_chunks / total_chunks if total_chunks > 0 else 0

    logger.info(f"  URL processing success rate: {success_rate:.2%}")
    logger.info(f"  Storage success rate: {storage_success_rate:.2%}")

    # Check if we meet minimum success criteria
    if success_rate >= 0.9:  # At least 90% of URLs should be processed successfully
        logger.info("✓ Pipeline completed successfully with high success rate")
        return True
    else:
        logger.warning("✗ Pipeline completed but with low success rate")
        return False


if __name__ == "__main__":
    main()


# Import additional modules for helper functions
import constants
import utils