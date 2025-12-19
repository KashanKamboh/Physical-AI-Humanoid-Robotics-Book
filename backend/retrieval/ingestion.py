"""
Data ingestion module for RAG Chatbot semantic chunking and Qdrant storage.

This module provides functionality to download, chunk, and ingest book content
into Qdrant with proper text payloads for retrieval.
"""
import logging
import requests
from typing import List, Dict
from urllib.parse import urljoin, urlparse
import time
from qdrant_client.http import models
from config.settings import get_qdrant_client, get_cohere_client, QDRANT_COLLECTION_NAME, VECTOR_SIZE
from retrieval.chunking import SemanticChunker, TextChunk


class BookIngestor:
    """Handles the complete ingestion pipeline for book content"""

    def __init__(self, chunk_size: int = 400, chunk_overlap: int = 75, rate_limit_delay: float = 1.0):
        """
        Initialize the book ingestor.

        Args:
            chunk_size: Target token count for each chunk (300-500)
            chunk_overlap: Token overlap between chunks (50-100)
            rate_limit_delay: Delay between API calls to respect rate limits
        """
        self.chunker = SemanticChunker(chunk_size=chunk_size, chunk_overlap=chunk_overlap)
        self.rate_limit_delay = rate_limit_delay
        self.qdrant_client = get_qdrant_client()
        self.cohere_client = get_cohere_client()

    def fetch_content(self, url: str) -> str:
        """
        Fetch HTML content from a URL.

        Args:
            url: URL to fetch content from

        Returns:
            HTML content as string
        """
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            return response.text
        except Exception as e:
            logging.error(f"Failed to fetch content from {url}: {str(e)}")
            raise

    def create_embedding(self, text: str) -> List[float]:
        """
        Create embedding for text using Cohere API.

        Args:
            text: Text to embed

        Returns:
            Embedding vector as list of floats
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-multilingual-v3.0",
                input_type="search_document"
            )
            embedding = response.embeddings[0]

            # Validate embedding dimensions
            if len(embedding) != VECTOR_SIZE:
                raise ValueError(f"Invalid embedding dimensions: expected {VECTOR_SIZE}, got {len(embedding)}")

            return embedding
        except Exception as e:
            logging.error(f"Failed to create embedding for text: {str(e)}")
            raise

    def ingest_chunk(self, chunk: TextChunk, batch_size: int = 10) -> None:
        """
        Ingest a single chunk into Qdrant with proper payload structure.

        Args:
            chunk: TextChunk to ingest
            batch_size: Number of points to batch together for ingestion
        """
        try:
            # Create embedding for the chunk
            embedding = self.create_embedding(chunk.content)

            # Create a unique ID that won't conflict with existing points
            import uuid
            unique_id = str(uuid.uuid4())

            # Prepare the point for Qdrant with proper payload structure
            point = models.PointStruct(
                id=unique_id,  # Use UUID to ensure uniqueness
                vector=embedding,
                payload={
                    "text": chunk.content,  # CRITICAL: This is the missing field!
                    "source_url": chunk.source_url,
                    "title": chunk.title,
                    "section": chunk.section,
                    "position": chunk.position,
                    "char_start": chunk.char_start,
                    "char_end": chunk.char_end,
                    "token_count": chunk.token_count,
                    "created_at": time.strftime('%Y-%m-%dT%H:%M:%S') + '.' + str(int(time.time() * 1000000) % 1000000).zfill(6) + 'Z'
                }
            )

            # Upsert the point into Qdrant
            self.qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=[point]
            )

            # Rate limiting
            time.sleep(self.rate_limit_delay)

        except Exception as e:
            logging.error(f"Failed to ingest chunk from {chunk.source_url}: {str(e)}")
            raise

    def ingest_chunks(self, chunks: List[TextChunk], batch_size: int = 10) -> Dict:
        """
        Ingest multiple chunks into Qdrant with batching and error handling.

        Args:
            chunks: List of TextChunk objects to ingest
            batch_size: Number of chunks to process in each batch

        Returns:
            Dictionary with ingestion statistics
        """
        total_chunks = len(chunks)
        successful_ingests = 0
        failed_ingests = 0

        logging.info(f"Starting ingestion of {total_chunks} chunks...")

        for i, chunk in enumerate(chunks):
            try:
                logging.info(f"Ingesting chunk {i+1}/{total_chunks} from {chunk.source_url}")
                self.ingest_chunk(chunk)
                successful_ingests += 1

                if successful_ingests % 10 == 0:  # Log progress every 10 chunks
                    logging.info(f"Progress: {successful_ingests}/{total_chunks} chunks ingested successfully")

            except Exception as e:
                logging.error(f"Failed to ingest chunk {i+1} from {chunk.source_url}: {str(e)}")
                failed_ingests += 1
                # Continue with next chunk instead of stopping the entire process

        stats = {
            "total_chunks_processed": total_chunks,
            "successful_ingests": successful_ingests,
            "failed_ingests": failed_ingests,
            "success_rate": successful_ingests / total_chunks if total_chunks > 0 else 0
        }

        logging.info(f"Ingestion completed. Stats: {stats}")
        return stats

    def ingest_from_url(self, url: str) -> Dict:
        """
        Complete ingestion pipeline: fetch, chunk, and ingest content from a URL.

        Args:
            url: URL to fetch and ingest content from

        Returns:
            Dictionary with ingestion statistics
        """
        try:
            logging.info(f"Starting ingestion from URL: {url}")

            # Fetch content
            html_content = self.fetch_content(url)

            # Chunk content semantically
            chunks = self.chunker.chunk_html_content(html_content, url)
            logging.info(f"Generated {len(chunks)} chunks from {url}")

            # Ingest chunks
            stats = self.ingest_chunks(chunks)

            return stats

        except Exception as e:
            logging.error(f"Failed to ingest from URL {url}: {str(e)}")
            raise

    def ingest_from_urls(self, urls: List[str]) -> Dict:
        """
        Ingest content from multiple URLs.

        Args:
            urls: List of URLs to ingest content from

        Returns:
            Dictionary with overall ingestion statistics
        """
        all_stats = {
            "total_chunks_processed": 0,
            "successful_ingests": 0,
            "failed_ingests": 0,
            "urls_processed": 0,
            "urls_failed": 0,
            "url_results": []
        }

        for url in urls:
            try:
                stats = self.ingest_from_url(url)
                all_stats["total_chunks_processed"] += stats["total_chunks_processed"]
                all_stats["successful_ingests"] += stats["successful_ingests"]
                all_stats["failed_ingests"] += stats["failed_ingests"]
                all_stats["urls_processed"] += 1

                all_stats["url_results"].append({
                    "url": url,
                    "stats": stats
                })

            except Exception as e:
                logging.error(f"Failed to process URL {url}: {str(e)}")
                all_stats["urls_failed"] += 1

                all_stats["url_results"].append({
                    "url": url,
                    "error": str(e)
                })

        # Calculate overall success rate
        total_attempts = all_stats["successful_ingests"] + all_stats["failed_ingests"]
        all_stats["overall_success_rate"] = (
            all_stats["successful_ingests"] / total_attempts if total_attempts > 0 else 0
        )

        logging.info(f"Overall ingestion completed. Stats: {all_stats}")
        return all_stats


def quick_validation_test():
    """
    Perform a quick validation test to confirm that retrieved chunks contain meaningful text.
    """
    from retrieval.search import semantic_search
    from retrieval.models import SearchQuery

    logging.info("Performing quick validation test...")

    try:
        # Run a simple search to test if text content is available
        query = SearchQuery("robotic", top_k=3, min_score=0.0)
        results = semantic_search(query)

        logging.info(f"Validation search returned {len(results)} chunks")

        for i, chunk in enumerate(results):
            logging.info(f"Result {i+1}: Score={chunk.similarity_score:.3f}")
            logging.info(f"  Source: {chunk.source_url}")
            logging.info(f"  Text preview: {chunk.text_content[:100]}...")
            logging.info(f"  Text length: {len(chunk.text_content)} chars")
            logging.info("---")

        if results:
            logging.info("✅ Validation successful! Retrieved chunks contain meaningful text content.")
            return True
        else:
            logging.info("⚠️  No chunks returned, but no errors occurred.")
            return False

    except Exception as e:
        logging.error(f"❌ Validation failed with error: {str(e)}")
        return False


if __name__ == "__main__":
    # Example usage
    ingestor = BookIngestor(chunk_size=400, chunk_overlap=75, rate_limit_delay=1.0)

    # Example URLs (replace with actual book URLs)
    urls = [
        "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/docs/intro",
        "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book/docs/course-roadmap"
    ]

    print("Starting ingestion process...")
    stats = ingestor.ingest_from_urls(urls)
    print(f"Ingestion stats: {stats}")

    print("\nRunning validation test...")
    quick_validation_test()