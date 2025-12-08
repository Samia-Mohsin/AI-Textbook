// UserProgress model for the Physical AI & Humanoid Robotics Book platform

class UserProgress {
  constructor(id, userId, moduleId, chapterId, status = 'not-started', progress = 0, lastAccessed = null, completedAt = null) {
    this.id = id;
    this.userId = userId; // Reference to the user
    this.moduleId = moduleId; // Reference to the module
    this.chapterId = chapterId; // Reference to the chapter
    this.status = status; // 'not-started', 'in-progress', 'completed'
    this.progress = progress; // Progress percentage (0-100)
    this.lastAccessed = lastAccessed || new Date();
    this.completedAt = completedAt; // When the chapter/module was completed (null if not completed)
    this.timeSpent = 0; // Time spent on this chapter/module in seconds
    this.notes = []; // Array of user notes for this chapter
    this.assessments = {}; // Object to store assessment results
  }

  // Update progress status
  updateStatus(newStatus) {
    this.status = newStatus;
    this.lastAccessed = new Date();

    if (newStatus === 'completed' && !this.completedAt) {
      this.completedAt = new Date();
    }

    // Update progress percentage based on status
    if (newStatus === 'completed') {
      this.progress = 100;
    } else if (newStatus === 'in-progress') {
      this.progress = Math.max(this.progress, 50); // At least 50% when in progress
    }
  }

  // Update progress percentage
  updateProgress(percentage) {
    this.progress = Math.min(100, Math.max(0, percentage)); // Clamp between 0 and 100
    this.lastAccessed = new Date();

    // Update status based on progress
    if (this.progress === 100) {
      this.status = 'completed';
      if (!this.completedAt) {
        this.completedAt = new Date();
      }
    } else if (this.progress > 0) {
      this.status = 'in-progress';
      this.completedAt = null;
    }
  }

  // Add time spent
  addTimeSpent(seconds) {
    this.timeSpent += seconds;
    this.lastAccessed = new Date();
  }

  // Add a note
  addNote(note) {
    this.notes.push({
      content: note,
      timestamp: new Date()
    });
    this.lastAccessed = new Date();
  }

  // Record assessment result
  recordAssessment(assessmentId, result) {
    this.assessments[assessmentId] = {
      result,
      completedAt: new Date()
    };
    this.lastAccessed = new Date();
  }

  // Get progress summary
  getSummary() {
    return {
      status: this.status,
      progress: this.progress,
      timeSpent: this.timeSpent,
      totalNotes: this.notes.length,
      totalAssessments: Object.keys(this.assessments).length,
      lastAccessed: this.lastAccessed,
      completedAt: this.completedAt
    };
  }

  // Check if prerequisite is satisfied
  static isPrerequisiteSatisfied(userProgress, prerequisiteChapterId) {
    const prereqProgress = userProgress.find(p => p.chapterId === prerequisiteChapterId);
    return prereqProgress && prereqProgress.status === 'completed';
  }
}

module.exports = UserProgress;