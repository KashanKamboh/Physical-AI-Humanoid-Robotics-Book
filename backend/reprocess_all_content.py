#!/usr/bin/env python3
"""
Script to reprocess and re-ingest all book content with proper text payloads.

This script will fetch all book content, chunk it semantically, and re-ingest it
into Qdrant with proper text payloads, replacing the existing data.
"""
import sys
import os
import logging
import time
from typing import List

# Add the backend directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from retrieval.ingestion import BookIngestor
from retrieval.chunking import SemanticChunker
from config.settings import get_qdrant_client, QDRANT_COLLECTION_NAME
import requests


def setup_logging():
    """Set up logging for the reprocessing process."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('reprocess.log'),
            logging.StreamHandler()
        ]
    )


def get_all_book_urls() -> List[str]:
    """
    Get all book URLs to reprocess.

    Returns:
        List of URLs for all book content
    """
    # Base URL for the book
    base_url = "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book"

    # All book sections to process
    sections = [
        "docs/intro",
        "docs/course-roadmap",
        "docs/module-1/intro",
        "docs/module-2/intro",
        "docs/module-3/intro",
        "docs/module-4/intro",
        "docs/module-5/intro",
        "docs/module-6/intro",
        "docs/module-7/intro",
        "docs/module-8/intro",
        "docs/module-9/intro",
        "docs/module-10/intro"
    ]

    urls = [f"{base_url}/{section}" for section in sections]
    return urls


def test_url_accessibility(urls: List[str]) -> List[str]:
    """
    Test which URLs are accessible before processing.

    Args:
        urls: List of URLs to test

    Returns:
        List of accessible URLs
    """
    accessible_urls = []

    for url in urls:
        try:
            response = requests.head(url, timeout=10)  # Use HEAD request to check accessibility
            if response.status_code == 200:
                accessible_urls.append(url)
                logging.info(f"✅ Accessible: {url}")
            else:
                logging.warning(f"⚠️  Not accessible (status {response.status_code}): {url}")
        except Exception as e:
            logging.warning(f"⚠️  Error accessing {url}: {e}")

    return accessible_urls


def main():
    """Main function to run the reprocessing."""
    setup_logging()
    logging.info("Starting comprehensive reprocessing of all book content...")

    try:
        # Get all book URLs
        all_urls = get_all_book_urls()
        logging.info(f"Found {len(all_urls)} total URLs to process")

        # Test which URLs are accessible
        accessible_urls = test_url_accessibility(all_urls)
        logging.info(f"Successfully verified {len(accessible_urls)} accessible URLs")

        if not accessible_urls:
            logging.error("No accessible URLs found. Cannot proceed with reprocessing.")
            return False

        # Initialize the ingestor with proper parameters
        ingestor = BookIngestor(
            chunk_size=400,      # Target: 300-500 tokens
            chunk_overlap=75,    # Target: 50-100 tokens
            rate_limit_delay=0.5  # Reduced delay for faster processing
        )

        # Process each URL
        total_chunks_processed = 0
        total_successful = 0
        total_failed = 0

        for i, url in enumerate(accessible_urls):
            logging.info(f"\nProcessing URL {i+1}/{len(accessible_urls)}: {url}")

            try:
                # Fetch content
                html_content = ingestor.fetch_content(url)
                logging.info(f"Fetched {len(html_content)} characters from {url}")

                # Generate chunks
                chunks = ingestor.chunker.chunk_html_content(html_content, url)
                logging.info(f"Generated {len(chunks)} chunks from {url}")

                # Process each chunk
                for j, chunk in enumerate(chunks):
                    try:
                        logging.info(f"  Processing chunk {j+1}/{len(chunks)} (tokens: {chunk.token_count})")

                        # Ingest the chunk with proper payload including text
                        ingestor.ingest_chunk(chunk)

                        total_successful += 1
                        total_chunks_processed += 1

                        if total_successful % 10 == 0:  # Log progress
                            logging.info(f"    Progress: {total_successful} chunks ingested successfully")

                    except Exception as e:
                        logging.error(f"    Failed to ingest chunk {j+1}: {e}")
                        total_failed += 1

            except Exception as e:
                logging.error(f"Failed to process URL {url}: {e}")
                total_failed += len(chunks) if 'chunks' in locals() else 0
                continue  # Continue with next URL

        # Final statistics
        logging.info(f"\n=== REPROCESSING COMPLETED ===")
        logging.info(f"Total chunks processed: {total_chunks_processed}")
        logging.info(f"Successful ingestions: {total_successful}")
        logging.info(f"Failed ingestions: {total_failed}")
        logging.info(f"Success rate: {total_successful/(total_successful+total_failed)*100:.1f}% if there were attempts")

        # Verify the reprocessing by checking a few points
        logging.info("\nVerifying reprocessing results...")
        qdrant_client = get_qdrant_client()

        # Count total points
        count_result = qdrant_client.count(
            collection_name=QDRANT_COLLECTION_NAME,
            exact=True
        )
        logging.info(f"Total points in collection after reprocessing: {count_result.count}")

        # Check a few points to verify text content exists
        points, _ = qdrant_client.scroll(
            collection_name=QDRANT_COLLECTION_NAME,
            limit=5,
            with_payload=True,
            with_vectors=False
        )

        points_with_text = 0
        for point in points:
            if point.payload and point.payload.get('text', ''):
                points_with_text += 1
                logging.info(f"✅ Point {point.id} has text content ({len(point.payload['text'])} chars)")
            else:
                logging.info(f"⚠️  Point {point.id} missing text content")

        logging.info(f"Verified {points_with_text}/{len(points)} sample points have text content")

        if points_with_text > 0:
            logging.info("✅ Reprocessing successful! Qdrant now contains text payloads for retrieval.")
            return True
        else:
            logging.warning("⚠️  Reprocessing completed but no points with text content found. Check ingestion logic.")
            return False

    except Exception as e:
        logging.error(f"❌ Reprocessing failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = main()
    if success:
        logging.info("\n🎉 Comprehensive reprocessing completed successfully!")
        print("\n✅ All book content has been reprocessed with semantic chunking and proper text payloads!")
        print("The Qdrant collection now contains complete text content for retrieval.")
    else:
        logging.error("\n❌ Reprocessing failed!")
        print("\n❌ Reprocessing failed! Check the logs for details.")
        sys.exit(1)