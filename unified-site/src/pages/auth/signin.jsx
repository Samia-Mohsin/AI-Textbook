import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';

// Use dynamic import to make this page client-only
const SigninPage = () => {
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
    password: ''
  });

  // Now that we're client-side, we can safely use the context
  const { login } = useAuth();

  // For Docusaurus, we'll use window.location for navigation
  const navigate = (path) => {
    window.location.hash = path;
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // In a real implementation, this would call your backend API
    // For now, we'll simulate the signin process
    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock login
      login({
        id: 'user_' + Date.now(),
        email: formData.email,
        name: 'Test User'
      }, 'mock_token_' + Date.now());

      navigate('/dashboard');
    } catch (error) {
      console.error('Signin error:', error);
      alert('Signin failed. Please try again.');
    }
  };

  return (
    <div className="auth-form">
      <h2>Sign In</h2>
      <form onSubmit={handleSubmit}>
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

        <button type="submit">Sign In</button>
      </form>

      <p style={{ marginTop: '1rem', textAlign: 'center' }}>
        Don't have an account? <a href="/auth/signup">Sign up</a>
      </p>
    </div>
  );
};

export default SigninPage;