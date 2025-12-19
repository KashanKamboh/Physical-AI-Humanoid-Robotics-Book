"""
Semantic chunking implementation for RAG Chatbot data ingestion.

This module provides functionality to parse and semantically chunk book content
with proper overlap and text payload for Qdrant ingestion.
"""
import re
import logging
from typing import List, Dict, Tuple
from dataclasses import dataclass
from bs4 import BeautifulSoup
import tiktoken
from config.settings import get_qdrant_client, QDRANT_COLLECTION_NAME, VECTOR_SIZE


@dataclass
class TextChunk:
    """Represents a semantically chunked piece of text with metadata"""
    content: str
    source_url: str
    title: str
    section: str
    position: int
    char_start: int
    char_end: int
    token_count: int


class SemanticChunker:
    """Handles semantic chunking of book content with proper overlap"""

    def __init__(self, chunk_size: int = 400, chunk_overlap: int = 75):
        """
        Initialize the semantic chunker.

        Args:
            chunk_size: Target token count for each chunk (300-500)
            chunk_overlap: Token overlap between chunks (50-100)
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")

    def clean_text(self, text: str) -> str:
        """
        Clean text content by removing extra whitespace and normalizing formatting.

        Args:
            text: Raw text content to clean

        Returns:
            Cleaned text content
        """
        # Remove extra whitespace and normalize line breaks
        text = re.sub(r'\s+', ' ', text)
        # Remove leading/trailing whitespace
        text = text.strip()
        return text

    def extract_structure(self, html_content: str, source_url: str) -> List[Dict]:
        """
        Extract structured content from HTML with semantic boundaries.

        Args:
            html_content: Raw HTML content
            source_url: Source URL of the content

        Returns:
            List of structured content blocks
        """
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Extract title
        title = soup.find('title')
        title = title.get_text().strip() if title else source_url.split('/')[-1]

        # Extract content sections based on semantic boundaries
        content_blocks = []

        # Look for headings and paragraphs
        for element in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'div', 'section', 'article']):
            text = element.get_text().strip()
            if not text or len(text) < 20:  # Skip very short content
                continue

            # Determine the semantic section based on heading hierarchy
            section = "default"
            parent = element.find_parent(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
            if parent:
                section = parent.get_text().strip()[:50]  # Limit section name length
            else:
                # Look for the closest heading above this element
                prev_heading = element.find_previous(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
                if prev_heading:
                    section = prev_heading.get_text().strip()[:50]

            content_blocks.append({
                'text': text,
                'section': section,
                'tag': element.name
            })

        # If no semantic structure found, use the whole content as one block
        if not content_blocks:
            full_text = soup.get_text().strip()
            if full_text:
                content_blocks.append({
                    'text': full_text,
                    'section': title[:50],
                    'tag': 'body'
                })

        return content_blocks

    def chunk_text(self, text: str, source_url: str, title: str) -> List[TextChunk]:
        """
        Semantically chunk text content with proper overlap.

        Args:
            text: Text content to chunk
            source_url: Source URL of the content
            title: Title of the document

        Returns:
            List of TextChunk objects
        """
        chunks = []

        # Clean the text first
        cleaned_text = self.clean_text(text)

        # Split into sentences for semantic boundaries
        sentences = re.split(r'[.!?]+\s+', cleaned_text)

        # Create chunks using a sliding window approach with semantic boundaries
        current_chunk = ""
        current_tokens = 0
        char_start = 0

        for i, sentence in enumerate(sentences):
            sentence = sentence.strip()
            if not sentence:
                continue

            sentence_tokens = len(self.tokenizer.encode(sentence))

            # If adding this sentence would exceed chunk size, save current chunk
            if current_tokens + sentence_tokens > self.chunk_size and current_chunk:
                # Add the chunk to results
                chunk = TextChunk(
                    content=current_chunk.strip(),
                    source_url=source_url,
                    title=title,
                    section=title[:50],
                    position=len(chunks),
                    char_start=char_start,
                    char_end=char_start + len(current_chunk),
                    token_count=current_tokens
                )
                chunks.append(chunk)

                # Start a new chunk with overlap
                overlap_tokens = min(self.chunk_overlap, current_tokens)
                overlap_start_idx = self._find_token_boundary(current_chunk, overlap_tokens)

                current_chunk = current_chunk[overlap_start_idx:] + " " + sentence + " "
                current_tokens = len(self.tokenizer.encode(current_chunk))
                char_start = char_start + overlap_start_idx

            else:
                current_chunk += sentence + " "
                current_tokens += sentence_tokens

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk = TextChunk(
                content=current_chunk.strip(),
                source_url=source_url,
                title=title,
                section=title[:50],
                position=len(chunks),
                char_start=char_start,
                char_end=char_start + len(current_chunk),
                token_count=current_tokens
            )
            chunks.append(chunk)

        return chunks

    def _find_token_boundary(self, text: str, target_tokens: int) -> int:
        """
        Find character index that corresponds to approximately target_tokens from the start.

        Args:
            text: Text to analyze
            target_tokens: Target number of tokens

        Returns:
            Character index that approximates the token boundary
        """
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= target_tokens:
            return 0

        # Decode back to get approximate character position
        approx_tokens = tokens[:target_tokens]
        approx_text = self.tokenizer.decode(approx_tokens)
        return len(approx_text)

    def chunk_html_content(self, html_content: str, source_url: str) -> List[TextChunk]:
        """
        Chunk HTML content semantically with proper structure awareness.

        Args:
            html_content: Raw HTML content to chunk
            source_url: Source URL of the content

        Returns:
            List of TextChunk objects
        """
        # Extract structured content
        content_blocks = self.extract_structure(html_content, source_url)

        all_chunks = []
        for block in content_blocks:
            text = block['text']
            section = block['section']

            # Chunk each semantic block
            chunks = self.chunk_text(text, source_url, section)

            # Update section information for each chunk
            for chunk in chunks:
                chunk.section = section

            all_chunks.extend(chunks)

        return all_chunks


def validate_ingestion():
    """
    Quick validation function to check if chunks are properly ingested with text content.

    Returns:
        ValidationResult indicating success or failure
    """
    from retrieval.models import ValidationResult
    from retrieval.exceptions import QdrantConnectionError

    try:
        qdrant_client = get_qdrant_client()

        # Get a sample of points to verify text content is present
        points, next_offset = qdrant_client.scroll(
            collection_name=QDRANT_COLLECTION_NAME,
            limit=5,
            with_payload=True,
            with_vectors=False
        )

        if not points:
            return ValidationResult(
                is_valid=False,
                errors=["No points found in collection"],
                warnings=[],
                metrics={"total_points": 0}
            )

        # Check if text field is present in payload
        missing_text_count = 0
        valid_text_count = 0

        for point in points:
            payload = point.payload
            if not payload or 'text' not in payload or not payload['text']:
                missing_text_count += 1
            else:
                valid_text_count += 1

        metrics = {
            "total_sampled": len(points),
            "points_with_text": valid_text_count,
            "points_without_text": missing_text_count
        }

        is_valid = valid_text_count > 0  # At least some points have text content
        errors = []
        if not is_valid:
            errors.append("No points in Qdrant contain text content")

        return ValidationResult(
            is_valid=is_valid,
            errors=errors,
            warnings=[],
            metrics=metrics
        )

    except Exception as e:
        logging.error(f"Error during ingestion validation: {str(e)}")
        return ValidationResult(
            is_valid=False,
            errors=[f"Ingestion validation failed: {str(e)}"],
            warnings=[],
            metrics={}
        )


if __name__ == "__main__":
    # Example usage
    chunker = SemanticChunker(chunk_size=400, chunk_overlap=75)

    # Example HTML content
    html_content = """
    <html>
    <head><title>Test Book Chapter</title></head>
    <body>
        <h1>Introduction to Robotics</h1>
        <p>Robotics is an interdisciplinary field that includes computer science, engineering, and other sciences.</p>
        <p>Modern robotics involves artificial intelligence, machine learning, and sophisticated control systems.</p>
        <h2>History of Robotics</h2>
        <p>The history of robotics spans several decades with significant advances in recent years.</p>
        <p>Early robots were simple mechanical devices, but modern robots are complex systems with advanced capabilities.</p>
    </body>
    </html>
    """

    chunks = chunker.chunk_html_content(html_content, "https://example.com/test")

    print(f"Generated {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1}: {chunk.token_count} tokens, {len(chunk.content)} chars")
        print(f"Content preview: {chunk.content[:100]}...")
        print()