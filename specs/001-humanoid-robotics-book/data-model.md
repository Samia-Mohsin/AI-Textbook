# Data Model: Physical AI & Humanoid Robotics Book

## Core Entities

### User
- **Fields**:
  - id (string, unique, required)
  - email (string, unique, required)
  - name (string, required)
  - role (enum: student, educator, developer)
  - skill_level (enum: beginner, intermediate, advanced)
  - created_at (timestamp)
  - last_login (timestamp)
  - preferences (object: language, personalization_enabled)

- **Validations**:
  - Email must be valid format
  - Role must be one of allowed values
  - Skill level must be specified

- **Relationships**:
  - One-to-many with UserProgress
  - One-to-many with UserAssessment

### Module
- **Fields**:
  - id (string, unique, required)
  - title (string, required)
  - description (string)
  - order (integer, required)
  - duration_weeks (integer, default: 4)
  - prerequisites (array of module IDs)
  - learning_outcomes (array of strings)

- **Validations**:
  - Title must be unique
  - Order must be positive integer
  - Duration must be positive

- **Relationships**:
  - One-to-many with Chapter
  - Many-to-many with UserProgress (through progress tracking)

### Chapter
- **Fields**:
  - id (string, unique, required)
  - module_id (string, required, foreign key to Module)
  - title (string, required)
  - content_path (string, required)
  - order (integer, required)
  - estimated_duration_minutes (integer)
  - concepts_covered (array of strings)
  - code_examples (array of file paths)

- **Validations**:
  - Title must be unique within module
  - Order must be positive integer
  - Module ID must reference existing module

- **Relationships**:
  - Many-to-one with Module
  - One-to-many with Exercise
  - One-to-many with UserProgress

### UserProgress
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required, foreign key to User)
  - module_id (string, required, foreign key to Module)
  - chapter_id (string, required, foreign key to Chapter)
  - status (enum: not_started, in_progress, completed)
  - progress_percentage (float, 0-100)
  - time_spent_seconds (integer)
  - last_accessed (timestamp)
  - completed_at (timestamp, optional)

- **Validations**:
  - Status must be one of allowed values
  - Progress percentage must be between 0-100
  - Time spent must be non-negative

- **Relationships**:
  - Many-to-one with User
  - Many-to-one with Module
  - Many-to-one with Chapter

### Assessment
- **Fields**:
  - id (string, unique, required)
  - chapter_id (string, required, foreign key to Chapter)
  - type (enum: quiz, practical, project)
  - title (string, required)
  - description (string)
  - questions (array of question objects)
  - passing_score (float, 0-100, default: 70)
  - max_attempts (integer, default: 3)

- **Validations**:
  - Type must be one of allowed values
  - Passing score must be between 0-100
  - Max attempts must be positive

- **Relationships**:
  - Many-to-one with Chapter
  - One-to-many with UserAssessment

### UserAssessment
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required, foreign key to User)
  - assessment_id (string, required, foreign key to Assessment)
  - attempt_number (integer, required)
  - score (float, 0-100)
  - answers (array of answer objects)
  - completed_at (timestamp)
  - passed (boolean)

- **Validations**:
  - Score must be between 0-100
  - Attempt number must be positive
  - Answers must match assessment question format

- **Relationships**:
  - Many-to-one with User
  - Many-to-one with Assessment

### ContentLocalization
- **Fields**:
  - id (string, unique, required)
  - content_id (string, required)
  - content_type (enum: module, chapter, assessment)
  - language_code (string, required, ISO 639-1)
  - translated_content (string, required)
  - status (enum: pending, in_progress, completed, reviewed)
  - translator_notes (string, optional)

- **Validations**:
  - Language code must be valid ISO format
  - Status must be one of allowed values
  - Content ID must exist in original language

- **Relationships**:
  - One-to-one with original content based on content_type

### AnalyticsSnapshot
- **Fields**:
  - id (string, unique, required)
  - user_id (string, required, foreign key to User)
  - event_type (enum: chapter_view, code_run, assessment_taken, video_play)
  - event_data (object, varies by event type)
  - timestamp (timestamp, required)
  - session_id (string)

- **Validations**:
  - Event type must be one of allowed values
  - Timestamp must be current or past

- **Relationships**:
  - Many-to-one with User

## State Transitions

### UserProgress States
- `not_started` → `in_progress` (when user starts chapter)
- `in_progress` → `completed` (when user completes chapter)
- `completed` → `in_progress` (if user revisits for review)

### Assessment States
- New assessment is created with default values
- User can attempt assessment (increases attempt_number)
- Assessment is scored and marked as passed/failed
- Assessment can be retaken if not passed and attempts remain

## Relationships Summary

- User has many UserProgress records
- User has many UserAssessment records
- Module has many Chapters
- Chapter belongs to one Module
- Chapter has many Exercises (not explicitly modeled, part of content)
- UserProgress connects User to Module and Chapter
- Assessment belongs to Chapter
- UserAssessment connects User to Assessment
- ContentLocalization provides translations for all content types
- AnalyticsSnapshot tracks user interactions