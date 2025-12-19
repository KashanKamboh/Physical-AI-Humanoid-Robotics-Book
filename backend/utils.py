"""
Utility functions for the RAG Ingestion Pipeline
"""
import re
from typing import List, Dict, Tuple
import uuid
from datetime import datetime


def generate_uuid() -> str:
    """Generate a UUID for chunk IDs."""
    return str(uuid.uuid4())


def get_iso_timestamp() -> str:
    """Generate ISO 8601 formatted timestamp."""
    return datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ')


def clean_text(text: str) -> str:
    """Clean text by removing extra whitespace and normalizing."""
    if not text:
        return ""

    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)
    # Remove leading/trailing whitespace
    text = text.strip()
    return text


def split_by_sentences(text: str, max_length: int = 512) -> List[str]:
    """Split text into chunks by sentences while respecting max_length."""
    sentences = re.split(r'(?<=[.!?])\s+', text)
    chunks = []
    current_chunk = ""

    for sentence in sentences:
        if len(current_chunk + sentence) <= max_length:
            current_chunk += sentence + " "
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = sentence + " "

    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks


def count_tokens(text: str) -> int:
    """Approximate token count using word splitting (simple approach)."""
    # This is a rough approximation - for more accurate token counting,
    # we could use a proper tokenizer like tiktoken
    if not text:
        return 0

    # Split on whitespace and punctuation
    tokens = re.findall(r'\b\w+\b', text)
    return len(tokens)


def validate_url(url: str) -> bool:
    """Validate if a string is a properly formatted URL."""
    import re
    url_pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return url_pattern.match(url) is not None