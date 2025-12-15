import re
from typing import List, Dict, Any
from config.settings import settings
import hashlib
from dataclasses import dataclass

@dataclass
class DocumentChunk:
    """Represents a chunk of document content with metadata"""
    id: str
    content: str
    source_url: str
    title: str
    chunk_index: int
    total_chunks: int
    content_hash: str
    metadata: Dict[str, Any]

class TextChunker:
    """Handles text chunking with configurable size and overlap"""

    def __init__(self, chunk_size: int = None, overlap: int = None):
        self.chunk_size = chunk_size or settings.CHUNK_SIZE
        self.overlap = overlap or settings.CHUNK_OVERLAP

    def chunk_content(self, content: str, source_url: str, title: str = "") -> List[DocumentChunk]:
        """Chunk content into semantic segments"""
        if not content.strip():
            return []

        # Split content into sentences to maintain semantic boundaries
        sentences = self._split_into_sentences(content)
        chunks = []
        chunk_index = 0

        current_chunk = ""
        current_sentence_idx = 0

        while current_sentence_idx < len(sentences):
            # Add sentences to current chunk until it reaches the desired size
            while current_sentence_idx < len(sentences):
                sentence = sentences[current_sentence_idx].strip()

                # Check if adding this sentence would exceed chunk size
                test_chunk = current_chunk + " " + sentence if current_chunk else sentence

                if len(test_chunk) <= self.chunk_size or not current_chunk:
                    current_chunk = test_chunk
                    current_sentence_idx += 1
                else:
                    break

            # If we have content in the current chunk, save it as a chunk
            if current_chunk.strip():
                chunk_id = f"{source_url}#{chunk_index}"
                content_hash = self._generate_content_hash(current_chunk)

                chunk = DocumentChunk(
                    id=chunk_id,
                    content=current_chunk.strip(),
                    source_url=source_url,
                    title=title,
                    chunk_index=chunk_index,
                    total_chunks=0,  # Will be updated later
                    content_hash=content_hash,
                    metadata={
                        'url': source_url,
                        'title': title,
                        'chunk_index': chunk_index,
                        'created_at': self._get_timestamp()
                    }
                )
                chunks.append(chunk)
                chunk_index += 1

            # If we've reached the end of sentences, break
            if current_sentence_idx >= len(sentences):
                break

            # Handle overlap: take some sentences from the end of the current chunk
            # to include at the beginning of the next chunk
            if self.overlap > 0 and current_sentence_idx < len(sentences):
                # Add overlap sentences to the next chunk
                overlap_sentences = self._get_overlap_sentences(current_chunk, sentences[current_sentence_idx:])

                # Start the next chunk with overlap content
                current_chunk = overlap_sentences
            else:
                current_chunk = ""

        # Update total chunks count
        for i, chunk in enumerate(chunks):
            chunks[i].total_chunks = len(chunks)

        return chunks

    def chunk_by_paragraphs(self, paragraphs: List[Dict], source_url: str, title: str = "") -> List[DocumentChunk]:
        """Chunk content by paragraphs with intelligent grouping"""
        chunks = []
        current_chunk_content = ""
        current_chunk_paragraphs = []
        chunk_index = 0

        for para in paragraphs:
            paragraph_text = para['text'].strip()
            if not paragraph_text:
                continue

            # Check if adding this paragraph would exceed chunk size
            test_chunk = current_chunk_content + "\n\n" + paragraph_text if current_chunk_content else paragraph_text

            if len(test_chunk) <= self.chunk_size and len(current_chunk_paragraphs) < 5:  # Limit paragraphs per chunk
                current_chunk_content = test_chunk
                current_chunk_paragraphs.append(para)
            else:
                # Save current chunk
                if current_chunk_content.strip():
                    chunk_id = f"{source_url}#para_{chunk_index}"
                    content_hash = self._generate_content_hash(current_chunk_content)

                    chunk = DocumentChunk(
                        id=chunk_id,
                        content=current_chunk_content.strip(),
                        source_url=source_url,
                        title=title,
                        chunk_index=chunk_index,
                        total_chunks=0,  # Will be updated later
                        content_hash=content_hash,
                        metadata={
                            'url': source_url,
                            'title': title,
                            'chunk_index': chunk_index,
                            'paragraph_types': [p['type'] for p in current_chunk_paragraphs],
                            'created_at': self._get_timestamp()
                        }
                    )
                    chunks.append(chunk)
                    chunk_index += 1

                # Start new chunk with current paragraph
                current_chunk_content = paragraph_text
                current_chunk_paragraphs = [para]

        # Add the last chunk if it has content
        if current_chunk_content.strip():
            chunk_id = f"{source_url}#para_{chunk_index}"
            content_hash = self._generate_content_hash(current_chunk_content)

            chunk = DocumentChunk(
                id=chunk_id,
                content=current_chunk_content.strip(),
                source_url=source_url,
                title=title,
                chunk_index=chunk_index,
                total_chunks=0,  # Will be updated later
                content_hash=content_hash,
                metadata={
                    'url': source_url,
                    'title': title,
                    'chunk_index': chunk_index,
                    'paragraph_types': [p['type'] for p in current_chunk_paragraphs],
                    'created_at': self._get_timestamp()
                }
            )
            chunks.append(chunk)

        # Update total chunks count
        for i, chunk in enumerate(chunks):
            chunks[i].total_chunks = len(chunks)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """Split text into sentences while preserving abbreviations"""
        # Handle common abbreviations to avoid false sentence breaks
        text = re.sub(r'\b([A-Z]\.)', r'SENTENCEABBR\1', text)
        text = re.sub(r'\b([A-Za-z]\.[A-Za-z]\.)', r'SENTENCEABBR\1', text)

        # Split on sentence endings
        sentences = re.split(r'[.!?]+\s+', text)

        # Restore abbreviations
        sentences = [s.replace('SENTENCEABBR', '') for s in sentences]

        return [s.strip() for s in sentences if s.strip()]

    def _get_overlap_sentences(self, chunk: str, remaining_sentences: List[str]) -> str:
        """Get sentences from the end of a chunk for overlap"""
        if not remaining_sentences:
            return ""

        # Get the last few sentences from the current chunk
        chunk_sentences = self._split_into_sentences(chunk)
        if not chunk_sentences:
            return ""

        # Take the last 1-2 sentences as overlap
        overlap_count = min(2, len(chunk_sentences))
        overlap_sentences = chunk_sentences[-overlap_count:]

        overlap_text = " ".join(overlap_sentences)

        # Add the first sentence from remaining if overlap is too short
        if len(overlap_text) < self.overlap and remaining_sentences:
            overlap_text += " " + remaining_sentences[0]

        return overlap_text

    def _generate_content_hash(self, content: str) -> str:
        """Generate a hash for content deduplication"""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()[:16]

    def _get_timestamp(self) -> str:
        """Get current timestamp"""
        from datetime import datetime
        return datetime.utcnow().isoformat()