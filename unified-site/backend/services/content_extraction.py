import requests
from bs4 import BeautifulSoup, NavigableString
from urllib.parse import urljoin, urlparse
import time
import logging
from typing import List, Dict, Optional, Tuple
from config.settings import settings

logger = logging.getLogger(__name__)

class ContentExtractor:
    """Extracts and cleans content from web pages"""

    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

    def fetch_page(self, url: str) -> Optional[str]:
        """Fetch page content with retry logic"""
        for attempt in range(settings.MAX_RETRIES):
            try:
                response = self.session.get(
                    url,
                    timeout=settings.REQUEST_TIMEOUT,
                    headers={'User-Agent': 'Mozilla/5.0 (compatible; BookBot/1.0)'}
                )
                response.raise_for_status()

                # Add delay to be respectful to the server
                time.sleep(settings.CRAWL_DELAY)

                return response.text
            except requests.RequestException as e:
                logger.warning(f"Attempt {attempt + 1} failed to fetch {url}: {str(e)}")
                if attempt == settings.MAX_RETRIES - 1:
                    logger.error(f"Failed to fetch {url} after {settings.MAX_RETRIES} attempts")
                    return None
                time.sleep(2 ** attempt)  # Exponential backoff

        return None

    def extract_content(self, html: str, url: str) -> Dict:
        """Extract clean content from HTML"""
        soup = BeautifulSoup(html, 'lxml')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Extract title
        title = ""
        if soup.title:
            title = soup.title.string.strip() if soup.title.string else ""

        # Extract main content - prioritize main content areas
        main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=lambda x: x and 'content' in x.lower())

        if not main_content:
            # If no main content area found, use the body
            main_content = soup.find('body')

        if main_content:
            # Extract text content, preserving paragraph structure
            paragraphs = []
            for element in main_content.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'code', 'pre']):
                text = element.get_text(separator=' ', strip=True)
                if text and len(text) > 10:  # Filter out very short text
                    element_type = element.name
                    paragraphs.append({
                        'text': text,
                        'type': element_type,
                        'level': int(element_type[1]) if element_type and element_type.startswith('h') else None
                    })
        else:
            # Fallback to extracting all paragraphs
            paragraphs = []
            for p in soup.find_all('p'):
                text = p.get_text(separator=' ', strip=True)
                if text and len(text) > 10:
                    paragraphs.append({
                        'text': text,
                        'type': 'p',
                        'level': None
                    })

        return {
            'url': url,
            'title': title,
            'content': ' '.join([p['text'] for p in paragraphs]),
            'paragraphs': paragraphs,
            'word_count': sum(len(p['text'].split()) for p in paragraphs)
        }

    def extract_links(self, html: str, base_url: str) -> List[str]:
        """Extract all valid links from the page"""
        soup = BeautifulSoup(html, 'lxml')
        links = []

        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)

            # Only include same-domain links that are not external
            if self._is_valid_internal_link(full_url, base_url):
                links.append(full_url)

        return list(set(links))  # Remove duplicates

    def _is_valid_internal_link(self, url: str, base_url: str) -> bool:
        """Check if a URL is a valid internal link"""
        parsed_url = urlparse(url)
        parsed_base = urlparse(base_url)

        # Same domain check
        if parsed_url.netloc != parsed_base.netloc:
            return False

        # Exclude certain file types and patterns
        excluded_extensions = ['.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe']
        excluded_patterns = ['/api/', '/admin/', '/login/', '/register/']

        url_lower = url.lower()
        if any(ext in url_lower for ext in excluded_extensions):
            return False

        if any(pattern in url_lower for pattern in excluded_patterns):
            return False

        return True


class SiteCrawler:
    """Crawls a website to extract all relevant pages"""

    def __init__(self):
        self.extractor = ContentExtractor()
        self.visited_urls = set()
        self.content_cache = {}

    def crawl_site(self, start_url: str, max_pages: int = 100) -> List[Dict]:
        """Crawl the site starting from start_url"""
        logger.info(f"Starting crawl from {start_url}")

        pages = []
        urls_to_visit = [start_url]

        while urls_to_visit and len(pages) < max_pages:
            current_url = urls_to_visit.pop(0)

            if current_url in self.visited_urls:
                continue

            self.visited_urls.add(current_url)
            logger.info(f"Crawling: {current_url}")

            html = self.extractor.fetch_page(current_url)
            if not html:
                continue

            # Extract content
            content = self.extractor.extract_content(html, current_url)
            if content['content'].strip():  # Only add pages with actual content
                pages.append(content)
                self.content_cache[current_url] = content

            # Extract and queue new URLs
            new_urls = self.extractor.extract_links(html, start_url)
            for url in new_urls:
                if url not in self.visited_urls and url not in urls_to_visit:
                    urls_to_visit.append(url)

            logger.info(f"Progress: {len(pages)} pages crawled, {len(urls_to_visit)} URLs queued")

        logger.info(f"Crawling completed. Total pages: {len(pages)}")
        return pages