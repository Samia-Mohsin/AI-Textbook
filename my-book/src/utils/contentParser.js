// Content parsing utility for the Physical AI & Humanoid Robotics Book platform
const fs = require('fs').promises;
const path = require('path');

class ContentParser {
  // Parse Markdown content and extract metadata
  static async parseMarkdown(filePath) {
    try {
      const content = await fs.readFile(filePath, 'utf8');

      // Extract frontmatter if present (YAML between ---)
      let frontmatter = {};
      let body = content;

      const frontmatterRegex = /^---\s*\n(.*?)\n---\s*\n/s;
      const frontmatterMatch = content.match(frontmatterRegex);

      if (frontmatterMatch) {
        const frontmatterStr = frontmatterMatch[1];
        frontmatter = this.parseYAML(frontmatterStr);
        body = content.slice(frontmatterMatch[0].length);
      }

      // Extract headings for table of contents
      const headings = this.extractHeadings(body);

      // Extract code blocks
      const codeBlocks = this.extractCodeBlocks(body);

      // Extract figures/images
      const figures = this.extractFigures(body);

      return {
        frontmatter,
        body,
        headings,
        codeBlocks,
        figures,
        wordCount: this.countWords(body),
        readingTime: this.estimateReadingTime(body)
      };
    } catch (error) {
      throw new Error(`Failed to parse content file: ${error.message}`);
    }
  }

  // Parse YAML frontmatter
  static parseYAML(yamlStr) {
    const result = {};
    const lines = yamlStr.split('\n');

    for (const line of lines) {
      const colonIndex = line.indexOf(':');
      if (colonIndex > 0) {
        const key = line.substring(0, colonIndex).trim();
        const value = line.substring(colonIndex + 1).trim();

        // Remove quotes if present
        const cleanValue = value.replace(/^["']|["']$/g, '');
        result[key] = cleanValue;
      }
    }

    return result;
  }

  // Extract headings from Markdown content
  static extractHeadings(content) {
    const headingRegex = /^(#{1,6})\s+(.+)$/gm;
    const headings = [];
    let match;

    while ((match = headingRegex.exec(content)) !== null) {
      headings.push({
        level: match[1].length,
        title: match[2].trim(),
        position: match.index
      });
    }

    return headings;
  }

  // Extract code blocks from Markdown content
  static extractCodeBlocks(content) {
    const codeBlockRegex = /```(\w*)\n([\s\S]*?)```/g;
    const codeBlocks = [];
    let match;

    while ((match = codeBlockRegex.exec(content)) !== null) {
      codeBlocks.push({
        language: match[1],
        code: match[2].trim(),
        position: match.index
      });
    }

    return codeBlocks;
  }

  // Extract figures/images from Markdown content
  static extractFigures(content) {
    const imageRegex = /!\[([^\]]*)\]\(([^)]+)\)/g;
    const figures = [];
    let match;

    while ((match = imageRegex.exec(content)) !== null) {
      figures.push({
        alt: match[1],
        src: match[2],
        position: match.index
      });
    }

    return figures;
  }

  // Count words in text
  static countWords(text) {
    return text.trim()
      .split(/\s+/)
      .filter(word => word.length > 0).length;
  }

  // Estimate reading time in minutes (average reading speed: 200 words per minute)
  static estimateReadingTime(text) {
    const wordsPerMinute = 200;
    const wordCount = this.countWords(text);
    return Math.ceil(wordCount / wordsPerMinute);
  }

  // Parse content with syntax highlighting
  static async parseWithSyntaxHighlighting(filePath) {
    const parsedContent = await this.parseMarkdown(filePath);

    // Add syntax highlighting info to code blocks
    parsedContent.codeBlocks = parsedContent.codeBlocks.map(block => ({
      ...block,
      highlighted: this.addSyntaxHighlighting(block.code, block.language)
    }));

    return parsedContent;
  }

  // Add syntax highlighting (simplified version - in practice, use a library like highlight.js)
  static addSyntaxHighlighting(code, language) {
    // This is a simplified version - in a real implementation,
    // you would use a syntax highlighting library
    return {
      code,
      language,
      highlighted: true
    };
  }

  // Validate content structure
  static validateContentStructure(parsedContent) {
    const errors = [];

    // Check for required headings
    const requiredHeadings = ['Learning Objectives', 'Summary'];
    for (const heading of requiredHeadings) {
      if (!parsedContent.headings.some(h => h.title.includes(heading))) {
        errors.push(`Missing required heading: ${heading}`);
      }
    }

    // Check for minimum word count
    if (parsedContent.wordCount < 500) {
      errors.push('Content is too short (minimum 500 words)');
    }

    // Check for code examples in technical chapters
    if (parsedContent.frontmatter && parsedContent.frontmatter.type === 'technical' && parsedContent.codeBlocks.length === 0) {
      errors.push('Technical chapter should contain code examples');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Get content statistics
  static getContentStats(parsedContent) {
    return {
      wordCount: parsedContent.wordCount,
      readingTime: parsedContent.readingTime,
      headingCount: parsedContent.headings.length,
      codeBlockCount: parsedContent.codeBlocks.length,
      figureCount: parsedContent.figures.length,
      hasFrontmatter: Object.keys(parsedContent.frontmatter).length > 0
    };
  }
}

module.exports = ContentParser;