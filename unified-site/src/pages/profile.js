import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { usePersonalization } from '../contexts/PersonalizationContext';

function ProfilePage() {
  const { siteConfig } = useDocusaurusContext();

  // Safely get context values, defaulting to safe values if context is not available
  let user, userPreferences, updateUserPreferences;

  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  try {
    const personalization = usePersonalization();
    userPreferences = personalization?.userPreferences || {};
    updateUserPreferences = personalization?.updatePreferences || (() => {});
  } catch {
    userPreferences = {};
    updateUserPreferences = () => {};
  }

  const [formData, setFormData] = useState(userPreferences || {});
  const [isEditing, setIsEditing] = useState(false);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (userPreferences) {
      setFormData(userPreferences);
    }
  }, [userPreferences]);

  const handleInputChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleArrayChange = (e, field) => {
    const { value, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [field]: checked
        ? [...prev[field], value]
        : prev[field].filter(item => item !== value)
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);

    try {
      await updateUserPreferences(formData);
      setIsEditing(false);
      alert('Profile updated successfully!');
    } catch (error) {
      console.error('Error updating profile:', error);
      alert('Error updating profile. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const programmingOptions = ['Python', 'C++', 'ROS', 'JavaScript', 'Java', 'Other'];
  const topicOptions = ['AI', 'Simulation', 'Control Systems', 'Hardware', 'Robotics', 'Other'];

  if (!user) {
    return (
      <Layout
        title={`Profile - ${siteConfig.title}`}
        description="User profile page">
        <div className="container padding-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="text--center padding-vert--md">
                <h2>Please sign in to view your profile</h2>
                <a href="/login" className="button button--primary button--lg">
                  Sign In
                </a>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout
      title={`Profile - ${siteConfig.title}`}
      description="User profile and preferences">
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>User Profile</h1>

            <div className="card">
              <div className="card__header">
                <h3>Account Information</h3>
              </div>
              <div className="card__body">
                <p><strong>Email:</strong> {user.email}</p>
                <p><strong>Name:</strong> {user.full_name || 'Not provided'}</p>
              </div>
            </div>

            <div className="card margin-top--lg">
              <div className="card__header">
                <h3>Learning Preferences</h3>
              </div>
              <div className="card__body">
                {isEditing ? (
                  <form onSubmit={handleSubmit}>
                    <div className="row">
                      <div className="col col--6">
                        <div className="form-group margin-bottom--md">
                          <label htmlFor="experience_level">Experience Level</label>
                          <select
                            className="form-control"
                            id="experience_level"
                            name="experience_level"
                            value={formData.experience_level}
                            onChange={handleInputChange}
                          >
                            <option value="beginner">Beginner</option>
                            <option value="intermediate">Intermediate</option>
                            <option value="advanced">Advanced</option>
                          </select>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="content_difficulty">Content Difficulty</label>
                          <select
                            className="form-control"
                            id="content_difficulty"
                            name="content_difficulty"
                            value={formData.content_difficulty}
                            onChange={handleInputChange}
                          >
                            <option value="easy">Easy</option>
                            <option value="moderate">Moderate</option>
                            <option value="difficult">Difficult</option>
                          </select>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="learning_style">Learning Style</label>
                          <select
                            className="form-control"
                            id="learning_style"
                            name="learning_style"
                            value={formData.learning_style}
                            onChange={handleInputChange}
                          >
                            <option value="visual">Visual</option>
                            <option value="auditory">Auditory</option>
                            <option value="reading">Reading/Writing</option>
                            <option value="kinesthetic">Kinesthetic</option>
                          </select>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="ai_ml_experience">AI/ML Experience</label>
                          <select
                            className="form-control"
                            id="ai_ml_experience"
                            name="ai_ml_experience"
                            value={formData.ai_ml_experience}
                            onChange={handleInputChange}
                          >
                            <option value="none">No experience</option>
                            <option value="beginner">Beginner</option>
                            <option value="intermediate">Intermediate</option>
                            <option value="advanced">Advanced</option>
                          </select>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="hardware_experience">Hardware Experience</label>
                          <select
                            className="form-control"
                            id="hardware_experience"
                            name="hardware_experience"
                            value={formData.hardware_experience}
                            onChange={handleInputChange}
                          >
                            <option value="none">No experience</option>
                            <option value="beginner">Beginner</option>
                            <option value="intermediate">Intermediate</option>
                            <option value="advanced">Advanced</option>
                          </select>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="robotics_kit_experience">Robotics Kit Experience</label>
                          <select
                            className="form-control"
                            id="robotics_kit_experience"
                            name="robotics_kit_experience"
                            value={formData.robotics_kit_experience}
                            onChange={handleInputChange}
                          >
                            <option value="none">None</option>
                            <option value="basic">Basic (e.g., Arduino, Raspberry Pi)</option>
                            <option value="intermediate">Intermediate (e.g., ROS robots)</option>
                            <option value="advanced">Advanced (e.g., custom robots)</option>
                          </select>
                        </div>
                      </div>

                      <div className="col col--6">
                        <div className="form-group margin-bottom--md">
                          <label htmlFor="learning_goals">Learning Goals</label>
                          <textarea
                            className="form-control"
                            id="learning_goals"
                            name="learning_goals"
                            rows="3"
                            value={formData.learning_goals || ''}
                            onChange={handleInputChange}
                            placeholder="What do you hope to achieve with this course?"
                          ></textarea>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label>Programming Languages Known</label>
                          {programmingOptions.map(lang => (
                            <div className="form-check" key={lang}>
                              <input
                                className="form-check-input"
                                type="checkbox"
                                id={`lang-${lang}`}
                                value={lang}
                                checked={formData.programming_languages?.includes(lang) || false}
                                onChange={(e) => handleArrayChange(e, 'programming_languages')}
                              />
                              <label className="form-check-label" htmlFor={`lang-${lang}`}>
                                {lang}
                              </label>
                            </div>
                          ))}
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label htmlFor="gpu_access" className="form-label">
                            <input
                              type="checkbox"
                              className="form-check-input"
                              id="gpu_access"
                              name="gpu_access"
                              checked={formData.gpu_access}
                              onChange={handleInputChange}
                            />
                            I have access to a GPU for AI training
                          </label>
                        </div>

                        <div className="form-group margin-bottom--md">
                          <label>Preferred Topics</label>
                          {topicOptions.map(topic => (
                            <div className="form-check" key={topic}>
                              <input
                                className="form-check-input"
                                type="checkbox"
                                id={`topic-${topic}`}
                                value={topic}
                                checked={formData.preferred_topics?.includes(topic) || false}
                                onChange={(e) => handleArrayChange(e, 'preferred_topics')}
                              />
                              <label className="form-check-label" htmlFor={`topic-${topic}`}>
                                {topic}
                              </label>
                            </div>
                          ))}
                        </div>
                      </div>
                    </div>

                    <div className="margin-top--lg">
                      <button
                        type="submit"
                        className="button button--primary"
                        disabled={loading}
                      >
                        {loading ? 'Updating...' : 'Save Changes'}
                      </button>
                      <button
                        type="button"
                        className="button button--secondary margin-left--sm"
                        onClick={() => setIsEditing(false)}
                        disabled={loading}
                      >
                        Cancel
                      </button>
                    </div>
                  </form>
                ) : (
                  <div>
                    <div className="row">
                      <div className="col col--6">
                        <p><strong>Experience Level:</strong> {formData.experience_level}</p>
                        <p><strong>Content Difficulty:</strong> {formData.content_difficulty}</p>
                        <p><strong>Learning Style:</strong> {formData.learning_style}</p>
                        <p><strong>AI/ML Experience:</strong> {formData.ai_ml_experience}</p>
                        <p><strong>Hardware Experience:</strong> {formData.hardware_experience}</p>
                        <p><strong>Robotics Kit Experience:</strong> {formData.robotics_kit_experience}</p>
                      </div>
                      <div className="col col--6">
                        <p><strong>Learning Goals:</strong> {formData.learning_goals || 'Not specified'}</p>
                        <p><strong>Programming Languages:</strong> {formData.programming_languages?.join(', ') || 'None specified'}</p>
                        <p><strong>GPU Access:</strong> {formData.gpu_access ? 'Yes' : 'No'}</p>
                        <p><strong>Preferred Topics:</strong> {formData.preferred_topics?.join(', ') || 'None specified'}</p>
                      </div>
                    </div>
                    <div className="margin-top--lg">
                      <button
                        className="button button--primary"
                        onClick={() => setIsEditing(true)}
                      >
                        Edit Profile
                      </button>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ProfilePage;