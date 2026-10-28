const { createProxyMiddleware } = require('http-proxy-middleware');
const { env } = require('process');

const target = env.REACT_APP_API_BASE_URL || 'http://localhost:5000';

module.exports = function (app) {
  app.use(
    createProxyMiddleware({
      // /ws: the remote shell's WebSocket relay (needs ws: true to proxy the upgrade).
      pathFilter: ['/api', '/ws'],
      target,
      ws: true,
      changeOrigin: true,
      secure: false,
      cookieDomainRewrite: '',
      cookiePathRewrite: '/',
    })
  );
};
