import cohere
from typing import List, Dict, Any, Optional
import logging
import time
from config.settings import settings
from services.text_chunking import DocumentChunk

logger = logging.getLogger(__name__)

class CohereEmbeddingService:
    """Service for generating embeddings using Cohere API"""

    def __init__(self):
        if not settings.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY is required for embedding service")

        self.client = cohere.Client(settings.COHERE_API_KEY)
        self.model = settings.COHERE_MODEL

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts"""
        if not texts:
            return []

        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for content indexing
            )

            return [embedding for embedding in response.embeddings]

        except cohere.CohereError as e:
            logger.error(f"Cohere API error: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error during embedding: {str(e)}")
            raise

    def generate_embeddings_batch(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """Generate embeddings in batches to respect API limits"""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            logger.info(f"Processing embedding batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

            try:
                batch_embeddings = self.generate_embeddings(batch)
                all_embeddings.extend(batch_embeddings)

                # Add small delay to be respectful to the API
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"Error processing batch {i//batch_size + 1}: {str(e)}")
                # Return partial results or raise based on requirements
                raise

        return all_embeddings

    def embed_document_chunks(self, chunks: List[DocumentChunk]) -> List[Dict[str, Any]]:
        """Generate embeddings for document chunks and return with metadata"""
        if not chunks:
            return []

        # Extract text content from chunks
        texts = [chunk.content for chunk in chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings_batch(texts)

        # Combine chunks with their embeddings
        result = []
        for i, chunk in enumerate(chunks):
            result.append({
                'id': chunk.id,
                'embedding': embeddings[i],
                'content': chunk.content,
                'metadata': chunk.metadata,
                'content_hash': chunk.content_hash
            })

        logger.info(f"Generated embeddings for {len(chunks)} chunks")
        return result


class EmbeddingValidator:
    """Validates embeddings and handles quality checks"""

    @staticmethod
    def validate_embedding(embedding: List[float], expected_dim: Optional[int] = None) -> bool:
        """Validate that an embedding is well-formed"""
        if not embedding:
            return False

        if expected_dim and len(embedding) != expected_dim:
            return False

        # Check for NaN or infinite values
        for value in embedding:
            if not isinstance(value, (int, float)) or value != value:  # NaN check: x != x for NaN
                return False

        return True

    @staticmethod
    def normalize_embedding(embedding: List[float]) -> List[float]:
        """Normalize embedding to unit vector"""
        import math

        magnitude = math.sqrt(sum(v * v for v in embedding))
        if magnitude == 0:
            return embedding

        return [v / magnitude for v in embedding]

    @staticmethod
    def calculate_similarity(vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors"""
        if len(vec1) != len(vec2):
            raise ValueError("Vectors must have the same dimension")

        # Dot product
        dot_product = sum(a * b for a, b in zip(vec1, vec2))

        # Magnitudes
        mag1 = sum(a * a for a in vec1) ** 0.5
        mag2 = sum(b * b for b in vec2) ** 0.5

        if mag1 == 0 or mag2 == 0:
            return 0.0

        return dot_product / (mag1 * mag2)