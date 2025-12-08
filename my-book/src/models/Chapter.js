// Chapter model for the Physical AI & Humanoid Robotics Book platform

class Chapter {
  constructor(id, moduleId, title, content, order, prerequisites = [], learningObjectives = [], createdAt, updatedAt) {
    this.id = id;
    this.moduleId = moduleId; // Reference to the parent module
    this.title = title;
    this.content = content; // Path to content file or content string
    this.order = order; // Order within the module
    this.prerequisites = prerequisites; // Array of chapter IDs that must be completed first
    this.learningObjectives = learningObjectives; // Array of learning objectives
    this.createdAt = createdAt || new Date();
    this.updatedAt = updatedAt || new Date();
    this.status = 'published'; // 'draft', 'published', 'archived'
    this.durationEstimate = 0; // Estimated time to complete in minutes
    this.codeExamples = []; // Array of code example file paths
    this.figures = []; // Array of figure/image paths
  }

  // Update chapter information
  updateInfo(title, content, learningObjectives) {
    if (title) this.title = title;
    if (content) this.content = content;
    if (learningObjectives) this.learningObjectives = learningObjectives;
    this.updatedAt = new Date();
  }

  // Add a prerequisite
  addPrerequisite(prerequisiteId) {
    if (!this.prerequisites.includes(prerequisiteId)) {
      this.prerequisites.push(prerequisiteId);
      this.updatedAt = new Date();
    }
  }

  // Add a learning objective
  addLearningObjective(objective) {
    if (!this.learningObjectives.includes(objective)) {
      this.learningObjectives.push(objective);
      this.updatedAt = new Date();
    }
  }

  // Add a code example
  addCodeExample(examplePath) {
    if (!this.codeExamples.includes(examplePath)) {
      this.codeExamples.push(examplePath);
      this.updatedAt = new Date();
    }
  }

  // Add a figure/image
  addFigure(figurePath) {
    if (!this.figures.includes(figurePath)) {
      this.figures.push(figurePath);
      this.updatedAt = new Date();
    }
  }

  // Set duration estimate
  setDurationEstimate(minutes) {
    this.durationEstimate = minutes;
    this.updatedAt = new Date();
  }

  // Get chapter statistics
  getStats() {
    return {
      totalPrerequisites: this.prerequisites.length,
      totalLearningObjectives: this.learningObjectives.length,
      totalCodeExamples: this.codeExamples.length,
      totalFigures: this.figures.length,
      durationEstimate: this.durationEstimate,
      createdAt: this.createdAt,
      updatedAt: this.updatedAt
    };
  }
}

module.exports = Chapter;