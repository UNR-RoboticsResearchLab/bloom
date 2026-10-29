const { createProxyMiddleware } = require('http-proxy-middleware');
const { env } = require('process');

const target = env.REACT_APP_API_BASE_URL || 'http://localhost:5000';

module.exports = function (app) {
  app.use(
    createProxyMiddleware({
      // /hubs: the remote shell's SignalR hubs (needs ws: true to proxy the WebSocket upgrade).
      pathFilter: ['/api', '/hubs'],
      target,
      ws: true,
      changeOrigin: true,
      secure: false,
      cookieDomainRewrite: '',
      cookiePathRewrite: '/',
    })
  );
};
