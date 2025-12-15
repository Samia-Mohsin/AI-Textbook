import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';

// Use dynamic import to make this page client-only
const SignupPage = () => {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // Don't render anything during server-side generation
  if (!isClient) {
    return (
      <div className="auth-form">
        <h2>Loading...</h2>
        <p>Please wait while the page loads.</p>
      </div>
    );
  }
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    programmingLanguages: [],
    aiMlExperience: '',
    hardwareExperience: '',
    learningGoals: '',
    gpuAccess: false,
    roboticsKitExperience: ''
  });

  const [selectedLanguages, setSelectedLanguages] = useState([]);

  // Now that we're client-side, we can safely use the context
  const { login } = useAuth();

  // For Docusaurus, we'll use window.location for navigation
  const navigate = (path) => {
    window.location.hash = path;
  };

  const programmingOptions = ['Python', 'C++', 'ROS', 'JavaScript', 'Rust', 'Other'];

  const handleLanguageChange = (lang) => {
    setSelectedLanguages(prev =>
      prev.includes(lang)
        ? prev.filter(l => l !== lang)
        : [...prev, lang]
    );
  };

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    const userData = {
      ...formData,
      programmingLanguages: selectedLanguages
    };

    // In a real implementation, this would call your backend API
    // For now, we'll simulate the signup process
    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock login
      login({
        id: 'user_' + Date.now(),
        email: formData.email,
        name: formData.name,
        preferences: {
          programmingLanguages: selectedLanguages,
          aiMlExperience: formData.aiMlExperience,
          hardwareExperience: formData.hardwareExperience,
          learningGoals: formData.learningGoals,
          gpuAccess: formData.gpuAccess,
          roboticsKitExperience: formData.roboticsKitExperience
        }
      }, 'mock_token_' + Date.now());

      navigate('/dashboard');
    } catch (error) {
      console.error('Signup error:', error);
      alert('Signup failed. Please try again.');
    }
  };

  return (
    <div className="auth-form">
      <h2>Create Your Account</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label>Programming Languages Known</label>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '10px' }}>
            {programmingOptions.map(lang => (
              <label key={lang} style={{ display: 'flex', alignItems: 'center' }}>
                <input
                  type="checkbox"
                  checked={selectedLanguages.includes(lang)}
                  onChange={() => handleLanguageChange(lang)}
                />
                {lang}
              </label>
            ))}
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="aiMlExperience">AI/ML Experience Level</label>
          <select
            id="aiMlExperience"
            name="aiMlExperience"
            value={formData.aiMlExperience}
            onChange={handleChange}
          >
            <option value="">Select your experience level</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
            <option value="expert">Expert</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardwareExperience">Hardware Experience</label>
          <textarea
            id="hardwareExperience"
            name="hardwareExperience"
            value={formData.hardwareExperience}
            onChange={handleChange}
            placeholder="Describe your experience with hardware platforms (e.g., Jetson, Raspberry Pi, Arduino, etc.)"
            rows="3"
          />
        </div>

        <div className="form-group">
          <label htmlFor="learningGoals">Learning Goals</label>
          <textarea
            id="learningGoals"
            name="learningGoals"
            value={formData.learningGoals}
            onChange={handleChange}
            placeholder="What are your goals with Physical AI and Humanoid Robotics?"
            rows="3"
          />
        </div>

        <div className="form-group">
          <label>
            <input
              type="checkbox"
              name="gpuAccess"
              checked={formData.gpuAccess}
              onChange={handleChange}
            />
            Do you have access to a GPU for training?
          </label>
        </div>

        <div className="form-group">
          <label htmlFor="roboticsKitExperience">Robotics Kit Experience</label>
          <input
            type="text"
            id="roboticsKitExperience"
            name="roboticsKitExperience"
            value={formData.roboticsKitExperience}
            onChange={handleChange}
            placeholder="Any experience with robotics kits like Jetson, TurtleBot, etc.?"
          />
        </div>

        <button type="submit">Sign Up</button>
      </form>

      <p style={{ marginTop: '1rem', textAlign: 'center' }}>
        Already have an account? <a href="/auth/signin">Sign in</a>
      </p>
    </div>
  );
};

export default SignupPage;