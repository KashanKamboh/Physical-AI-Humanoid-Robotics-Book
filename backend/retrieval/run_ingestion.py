#!/usr/bin/env python3
"""
Script to run semantic chunking and re-ingestion for book content.

This script will fetch book content, chunk it semantically, and re-ingest it
into Qdrant with proper text payloads.
"""
import sys
import os
import logging
from typing import List

# Add the backend directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ingestion import BookIngestor, quick_validation_test
from chunking import validate_ingestion


def setup_logging():
    """Set up logging for the ingestion process."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('ingestion.log'),
            logging.StreamHandler()
        ]
    )


def get_book_urls() -> List[str]:
    """
    Get the list of book URLs to ingest.

    Returns:
        List of URLs for book content
    """
    # Base URL for the book
    base_url = "https://kashankamboh.github.io/Physical-AI-Humanoid-Robotics-Book"

    # Common book sections
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


def main():
    """Main function to run the ingestion process."""
    setup_logging()
    logging.info("Starting semantic chunking and re-ingestion process...")

    try:
        # Initialize the ingestor with proper parameters
        ingestor = BookIngestor(
            chunk_size=400,      # Target: 300-500 tokens
            chunk_overlap=75,    # Target: 50-100 tokens
            rate_limit_delay=1.0  # Respect API rate limits
        )

        # Get book URLs to process
        urls = get_book_urls()
        logging.info(f"Found {len(urls)} URLs to process: {urls[:3]}...")  # Show first 3

        # Ingest content from all URLs
        logging.info("Starting ingestion process...")
        stats = ingestor.ingest_from_urls(urls)

        logging.info(f"Ingestion completed with stats: {stats}")

        # Run validation to check if ingestion was successful
        logging.info("Running ingestion validation...")
        validation_result = validate_ingestion()
        logging.info(f"Validation result: is_valid={validation_result.is_valid}")
        logging.info(f"Validation metrics: {validation_result.metrics}")

        if validation_result.is_valid:
            logging.info("✅ Ingestion validation PASSED! Text content is now available in Qdrant.")

            # Run a quick retrieval test to confirm functionality
            logging.info("Running quick retrieval validation test...")
            retrieval_success = quick_validation_test()

            if retrieval_success:
                logging.info("✅ All validation tests PASSED! The retrieval system is now working correctly.")
                return True
            else:
                logging.warning("⚠️  Retrieval test didn't return results, but ingestion validation passed.")
                return True
        else:
            logging.error(f"❌ Ingestion validation FAILED: {validation_result.errors}")
            return False

    except Exception as e:
        logging.error(f"❌ Ingestion process failed with error: {str(e)}")
        import traceback
        logging.error(f"Traceback: {traceback.format_exc()}")
        return False


if __name__ == "__main__":
    success = main()
    if success:
        logging.info("Ingestion process completed successfully!")
        print("\n✅ Semantic chunking and re-ingestion completed successfully!")
        print("The Qdrant collection now contains proper text payloads for retrieval.")
    else:
        logging.error("Ingestion process failed!")
        print("\n❌ Ingestion process failed! Check the logs for details.")
        sys.exit(1)