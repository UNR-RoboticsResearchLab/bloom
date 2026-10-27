import { resolveApiBaseUrl } from './ApiClientContext';

test('tunneled study uses HTTPS origin without exposing a backend port', () => {
  expect(resolveApiBaseUrl('same-origin', {
    origin: 'https://dev-bloom.unr.dev', protocol: 'https:', hostname: 'dev-bloom.unr.dev',
  })).toBe('https://dev-bloom.unr.dev');
});

test('combined local study preserves port 8081', () => {
  expect(resolveApiBaseUrl('same-origin', {
    origin: 'http://localhost:8081', protocol: 'http:', hostname: 'localhost',
  })).toBe('http://localhost:8081');
});

test('existing separate frontend development and explicit API settings still work', () => {
  const location = { protocol: 'http:', hostname: 'localhost' };
  expect(resolveApiBaseUrl(undefined, location)).toBe('http://localhost:8080');
  expect(resolveApiBaseUrl('https://bloom.unr.dev', location)).toBe('https://bloom.unr.dev');
});
