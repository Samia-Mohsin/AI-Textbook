// Module model for the Physical AI & Humanoid Robotics Book platform

class Module {
  constructor(id, title, description, order, chapters = [], createdAt, updatedAt) {
    this.id = id;
    this.title = title;
    this.description = description;
    this.order = order; // Order in the curriculum (1-4 for the 4 modules)
    this.chapters = chapters; // Array of chapter IDs or chapter objects
    this.createdAt = createdAt || new Date();
    this.updatedAt = updatedAt || new Date();
    this.status = 'published'; // 'draft', 'published', 'archived'
  }

  // Add a chapter to the module
  addChapter(chapterId) {
    if (!this.chapters.includes(chapterId)) {
      this.chapters.push(chapterId);
      this.updatedAt = new Date();
    }
  }

  // Remove a chapter from the module
  removeChapter(chapterId) {
    this.chapters = this.chapters.filter(id => id !== chapterId);
    this.updatedAt = new Date();
  }

  // Update module information
  updateInfo(title, description) {
    if (title) this.title = title;
    if (description) this.description = description;
    this.updatedAt = new Date();
  }

  // Get module statistics
  getStats() {
    return {
      totalChapters: this.chapters.length,
      createdAt: this.createdAt,
      updatedAt: this.updatedAt
    };
  }
}

module.exports = Module;