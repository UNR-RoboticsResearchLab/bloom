import React from 'react';
import ReactDOM from 'react-dom';
import { MemoryRouter } from 'react-router-dom';
import { ThemeProvider } from '@material-tailwind/react';
import App from './App';
import { ApiClientProvider } from './context/ApiClientContext';
import { RobotPairingProvider } from './context/RobotPairingContext';

// Mirrors the provider tree index.js actually wraps <App/> in (BrowserRouter swapped for
// MemoryRouter, which doesn't need a real browser history). Rendering <App/> bare -- as this
// test used to -- throws as soon as anything on the initial route calls useApiClient()/
// useRobotPairing() (e.g. NavMenu), since those hooks require their provider to be present.
it('renders without crashing', async () => {
  const div = document.createElement('div');
  ReactDOM.render(
    <ThemeProvider>
      <MemoryRouter>
        <ApiClientProvider>
          <RobotPairingProvider>
            <App />
          </RobotPairingProvider>
        </ApiClientProvider>
      </MemoryRouter>
    </ThemeProvider>, div);
  await new Promise(resolve => setTimeout(resolve, 1000));
});
