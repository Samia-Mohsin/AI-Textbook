import qdrant_client
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PayloadSchemaType
from typing import List, Dict, Any, Optional
import logging
from config.settings import settings

logger = logging.getLogger(__name__)

class QdrantVectorStorage:
    """Service for storing and retrieving vectors in Qdrant"""

    def __init__(self):
        # Initialize Qdrant client
        self.client = qdrant_client.QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10.0
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME

        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """Create the collection with appropriate vector configuration"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception:
            # Collection doesn't exist, create it
            # Get embedding dimension by calling Cohere API with a test text
            # For now, we'll use the default dimension for Cohere embeddings
            # Multilingual v3 embeddings have 1024 dimensions
            embedding_dim = 1024

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=embedding_dim,
                    distance=Distance.COSINE
                )
            )

            # Create payload indexes for efficient filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="url",
                field_schema=PayloadSchemaType.KEYWORD
            )

            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="content_hash",
                field_schema=PayloadSchemaType.KEYWORD
            )

            logger.info(f"Created collection '{self.collection_name}' with {embedding_dim} dimensions")

    def store_embeddings(self, embedding_records: List[Dict[str, Any]], batch_size: int = 64) -> bool:
        """Store embedding records in Qdrant"""
        if not embedding_records:
            logger.warning("No embedding records to store")
            return True

        try:
            # Process in batches for efficiency
            for i in range(0, len(embedding_records), batch_size):
                batch = embedding_records[i:i + batch_size]
                logger.info(f"Storing batch {i//batch_size + 1}/{(len(embedding_records)-1)//batch_size + 1}")

                # Prepare points for upsert
                points = []
                for record in batch:
                    point = models.PointStruct(
                        id=record['id'],
                        vector=record['embedding'],
                        payload={
                            'content': record['content'],
                            'url': record['metadata']['url'],
                            'title': record['metadata']['title'],
                            'chunk_index': record['metadata']['chunk_index'],
                            'content_hash': record['content_hash'],
                            'created_at': record['metadata']['created_at']
                        }
                    )
                    points.append(point)

                # Upsert points to handle duplicates
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points,
                    wait=True  # Wait for operation to complete
                )

            logger.info(f"Successfully stored {len(embedding_records)} embedding records")
            return True

        except Exception as e:
            logger.error(f"Error storing embeddings: {str(e)}")
            return False

    def search_similar(self, query_embedding: List[float], top_k: int = 4, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """Search for similar vectors in the collection"""
        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    if key == 'url':
                        filter_conditions.append(
                            models.FieldCondition(
                                key='url',
                                match=models.MatchValue(value=value)
                            )
                        )
                    elif key == 'content_hash':
                        filter_conditions.append(
                            models.FieldCondition(
                                key='content_hash',
                                match=models.MatchValue(value=value)
                            )
                        )

                if filter_conditions:
                    qdrant_filters = models.Filter(must=filter_conditions)

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                query_filter=qdrant_filters
            )

            # Format results
            results = []
            for hit in search_results:
                results.append({
                    'id': hit.id,
                    'content': hit.payload.get('content', ''),
                    'url': hit.payload.get('url', ''),
                    'title': hit.payload.get('title', ''),
                    'chunk_index': hit.payload.get('chunk_index'),
                    'similarity_score': hit.score,
                    'content_hash': hit.payload.get('content_hash'),
                    'metadata': hit.payload
                })

            return results

        except Exception as e:
            logger.error(f"Error searching for similar vectors: {str(e)}")
            return []

    def check_content_exists(self, content_hash: str) -> bool:
        """Check if content with given hash already exists in the collection"""
        try:
            hits = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_hash",
                            match=models.MatchValue(value=content_hash)
                        )
                    ]
                ),
                limit=1
            )

            return len(hits[0]) > 0 if hits else False

        except Exception as e:
            logger.error(f"Error checking content existence: {str(e)}")
            return False

    def delete_by_content_hash(self, content_hash: str) -> bool:
        """Delete vectors with specific content hash"""
        try:
            # Find points with the content hash
            hits = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_hash",
                            match=models.MatchValue(value=content_hash)
                        )
                    ]
                ),
                limit=1000  # Adjust as needed
            )

            if hits and hits[0]:
                # Extract IDs to delete
                ids_to_delete = [hit.id for hit in hits[0]]

                # Delete the points
                self.client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=ids_to_delete
                    )
                )

                logger.info(f"Deleted {len(ids_to_delete)} vectors with hash {content_hash}")
                return True
            else:
                logger.info(f"No vectors found with hash {content_hash}")
                return True

        except Exception as e:
            logger.error(f"Error deleting vectors with hash {content_hash}: {str(e)}")
            return False

    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection"""
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'name': collection_info.config.params.vectors.size,
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'point_count': collection_info.points_count
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return {}

    def clear_collection(self) -> bool:
        """Clear all vectors from the collection (use with caution!)"""
        try:
            # Delete all points by using a filter that matches everything
            # Since Qdrant doesn't have a direct clear method, we'll delete in batches
            while True:
                # Get first 1000 points
                hits = self.client.scroll(
                    collection_name=self.collection_name,
                    limit=1000
                )

                if not hits or not hits[0]:
                    break  # No more points to delete

                # Delete these points
                ids_to_delete = [hit.id for hit in hits[0]]
                self.client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=ids_to_delete
                    )
                )

                logger.info(f"Deleted batch of {len(ids_to_delete)} points")

            logger.info("Collection cleared successfully")
            return True
        except Exception as e:
            logger.error(f"Error clearing collection: {str(e)}")
            return False