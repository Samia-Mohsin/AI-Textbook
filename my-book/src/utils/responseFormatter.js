// API response formatting utility for the Physical AI & Humanoid Robotics Book platform

const formatSuccessResponse = (data, message = 'Success', statusCode = 200) => {
  return {
    success: true,
    statusCode,
    message,
    data
  };
};

const formatErrorResponse = (error, message = 'Error occurred', statusCode = 400) => {
  return {
    success: false,
    statusCode,
    message,
    error: error.message || error
  };
};

const formatPaginatedResponse = (data, pagination, message = 'Success', statusCode = 200) => {
  return {
    success: true,
    statusCode,
    message,
    data,
    pagination
  };
};

// Middleware to format API responses
const formatResponse = (req, res, next) => {
  res.apiSuccess = (data, message = 'Success', statusCode = 200) => {
    res.status(statusCode).json(formatSuccessResponse(data, message, statusCode));
  };

  res.apiError = (error, message = 'Error occurred', statusCode = 400) => {
    res.status(statusCode).json(formatErrorResponse(error, message, statusCode));
  };

  next();
};

module.exports = {
  formatSuccessResponse,
  formatErrorResponse,
  formatPaginatedResponse,
  formatResponse
};