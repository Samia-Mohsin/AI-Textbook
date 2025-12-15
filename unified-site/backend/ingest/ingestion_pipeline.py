import logging
from typing import List, Dict, Any
from config.settings import settings
from services.content_extraction import SiteCrawler
from services.text_chunking import TextChunker, DocumentChunk
from services.embedding_service import CohereEmbeddingService
from services.vector_storage import QdrantVectorStorage

logger = logging.getLogger(__name__)

class IngestionPipeline:
    """Main orchestrator for the content ingestion pipeline"""

    def __init__(self):
        # Initialize all services
        self.crawler = SiteCrawler()
        self.chunker = TextChunker(
            chunk_size=settings.CHUNK_SIZE,
            overlap=settings.CHUNK_OVERLAP
        )
        self.embedding_service = CohereEmbeddingService()
        self.vector_storage = QdrantVectorStorage()

    def run_ingestion(self, urls: List[str], max_pages: int = 100) -> Dict[str, Any]:
        """Run the complete ingestion pipeline"""
        logger.info(f"Starting ingestion pipeline for {len(urls)} URLs")
        settings.validate()  # Validate configuration

        results = {
            'status': 'started',
            'urls_processed': 0,
            'pages_crawled': 0,
            'chunks_created': 0,
            'embeddings_generated': 0,
            'vectors_stored': 0,
            'errors': [],
            'details': []
        }

        try:
            # Phase 1: Content Crawling
            logger.info("Phase 1: Starting content crawling...")
            all_pages = []
            for url in urls:
                try:
                    pages = self.crawler.crawl_site(url, max_pages=max_pages//len(urls) + 1)
                    all_pages.extend(pages)
                    results['urls_processed'] += 1
                    logger.info(f"Crawled {len(pages)} pages from {url}")
                except Exception as e:
                    error_msg = f"Error crawling {url}: {str(e)}"
                    logger.error(error_msg)
                    results['errors'].append(error_msg)

            results['pages_crawled'] = len(all_pages)
            logger.info(f"Completed crawling: {len(all_pages)} pages total")

            if not all_pages:
                logger.warning("No pages were successfully crawled")
                results['status'] = 'completed_with_errors'
                return results

            # Phase 2: Content Chunking
            logger.info("Phase 2: Starting content chunking...")
            all_chunks = []
            for page in all_pages:
                try:
                    # Chunk by paragraphs for better semantic boundaries
                    chunks = self.chunker.chunk_by_paragraphs(
                        page['paragraphs'],
                        page['url'],
                        page['title']
                    )
                    all_chunks.extend(chunks)
                except Exception as e:
                    error_msg = f"Error chunking content from {page['url']}: {str(e)}"
                    logger.error(error_msg)
                    results['errors'].append(error_msg)

            results['chunks_created'] = len(all_chunks)
            logger.info(f"Completed chunking: {len(all_chunks)} chunks created")

            if not all_chunks:
                logger.warning("No chunks were created from crawled content")
                results['status'] = 'completed_with_errors'
                return results

            # Phase 3: Duplicate Detection
            logger.info("Phase 3: Checking for duplicates...")
            unique_chunks = []
            duplicate_count = 0

            for chunk in all_chunks:
                if not self.vector_storage.check_content_exists(chunk.content_hash):
                    unique_chunks.append(chunk)
                else:
                    duplicate_count += 1
                    logger.info(f"Duplicate content detected: {chunk.id}")

            logger.info(f"Duplicates found: {duplicate_count}, Unique chunks: {len(unique_chunks)}")

            if not unique_chunks:
                logger.info("No new content to process (all duplicates)")
                results['status'] = 'completed_no_new_content'
                return results

            # Phase 4: Embedding Generation
            logger.info("Phase 4: Generating embeddings...")
            try:
                embedding_records = self.embedding_service.embed_document_chunks(unique_chunks)
                results['embeddings_generated'] = len(embedding_records)
                logger.info(f"Generated {len(embedding_records)} embeddings")
            except Exception as e:
                error_msg = f"Error generating embeddings: {str(e)}"
                logger.error(error_msg)
                results['errors'].append(error_msg)
                results['status'] = 'failed_at_embedding'
                return results

            # Phase 5: Vector Storage
            logger.info("Phase 5: Storing vectors in Qdrant...")
            try:
                success = self.vector_storage.store_embeddings(embedding_records)
                if success:
                    results['vectors_stored'] = len(embedding_records)
                    logger.info(f"Successfully stored {len(embedding_records)} vectors")
                else:
                    logger.error("Failed to store vectors in Qdrant")
                    results['status'] = 'failed_at_storage'
                    return results
            except Exception as e:
                error_msg = f"Error storing vectors: {str(e)}"
                logger.error(error_msg)
                results['errors'].append(error_msg)
                results['status'] = 'failed_at_storage'
                return results

            # Phase 6: Final Status
            results['status'] = 'completed' if not results['errors'] else 'completed_with_errors'
            logger.info(f"Ingestion pipeline completed with status: {results['status']}")

        except Exception as e:
            error_msg = f"Critical error in ingestion pipeline: {str(e)}"
            logger.error(error_msg)
            results['errors'].append(error_msg)
            results['status'] = 'failed'

        return results

    def incremental_update(self, urls: List[str], max_pages: int = 100) -> Dict[str, Any]:
        """Run incremental update that only processes new or changed content"""
        logger.info("Starting incremental update pipeline...")

        # For incremental updates, we'll crawl and check each page against existing content
        results = {
            'status': 'started',
            'urls_processed': 0,
            'pages_crawled': 0,
            'new_pages': 0,
            'updated_pages': 0,
            'unchanged_pages': 0,
            'chunks_created': 0,
            'embeddings_generated': 0,
            'vectors_stored': 0,
            'errors': [],
            'details': []
        }

        try:
            for url in urls:
                try:
                    # Crawl the specific URL
                    pages = self.crawler.crawl_site(url, max_pages=1)  # Just the main page for update
                    results['urls_processed'] += 1

                    for page in pages:
                        results['pages_crawled'] += 1

                        # Create chunks for this page
                        chunks = self.chunker.chunk_by_paragraphs(
                            page['paragraphs'],
                            page['url'],
                            page['title']
                        )

                        # Check which chunks are new vs. existing
                        new_chunks = []
                        for chunk in chunks:
                            if not self.vector_storage.check_content_exists(chunk.content_hash):
                                new_chunks.append(chunk)
                            else:
                                results['unchanged_pages'] += 1  # Count at chunk level as page update status

                        if len(new_chunks) == 0:
                            # Page hasn't changed
                            results['unchanged_pages'] += 1
                            logger.info(f"Page unchanged: {page['url']}")
                        elif len(new_chunks) == len(chunks):
                            # Entire page is new
                            results['new_pages'] += 1
                            logger.info(f"New page: {page['url']}")
                        else:
                            # Page has been updated
                            results['updated_pages'] += 1
                            logger.info(f"Updated page: {page['url']}")

                        # Process new chunks
                        if new_chunks:
                            # Generate embeddings for new chunks
                            embedding_records = self.embedding_service.embed_document_chunks(new_chunks)
                            results['chunks_created'] += len(new_chunks)
                            results['embeddings_generated'] += len(embedding_records)

                            # Store new embeddings
                            success = self.vector_storage.store_embeddings(embedding_records)
                            if success:
                                results['vectors_stored'] += len(embedding_records)

                except Exception as e:
                    error_msg = f"Error processing {url}: {str(e)}"
                    logger.error(error_msg)
                    results['errors'].append(error_msg)

            results['status'] = 'completed' if not results['errors'] else 'completed_with_errors'
            logger.info(f"Incremental update completed: {results}")

        except Exception as e:
            error_msg = f"Critical error in incremental update: {str(e)}"
            logger.error(error_msg)
            results['errors'].append(error_msg)
            results['status'] = 'failed'

        return results

    def get_pipeline_status(self) -> Dict[str, Any]:
        """Get current status of the ingestion system"""
        collection_info = self.vector_storage.get_collection_info()

        return {
            'pipeline_status': 'ready',
            'qdrant_collection': collection_info,
            'configuration_valid': True,
            'services_connected': {
                'cohere': bool(settings.COHERE_API_KEY),
                'qdrant': True,  # We tested connection during init
            }
        }