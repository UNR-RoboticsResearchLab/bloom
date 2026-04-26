const createProxyMiddleware = require('http-proxy-middleware');
const { env } = require('process');

const target = env.REACT_APP_API_BASE_URL || 'http://localhost:5000';

module.exports = function (app) {
  app.use(
    createProxyMiddleware('/api', {
      target,
      changeOrigin: true,
      secure: false,
      cookieDomainRewrite: '',
      cookiePathRewrite: '/',
      logLevel: 'debug',
    })
  );
};
