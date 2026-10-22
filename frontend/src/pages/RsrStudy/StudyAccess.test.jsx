import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import { MemoryRouter, Route, Routes } from 'react-router-dom';
import StudyAccess from './StudyAccess';
const act = React.act || legacyAct;
const mockApi = { getStudySession: jest.fn() };
jest.mock('../../context/ApiClientContext', () => ({ useApiClient: () => mockApi }));
let host, root;
beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  host = document.createElement('div'); document.body.appendChild(host); root = createRoot(host);
  localStorage.setItem('authUser', JSON.stringify({ id:'forged', role:'admin' }));
});
afterEach(async () => { await act(async () => root.unmount()); host.remove(); localStorage.clear(); });
async function render() {
  await act(async () => root.render(<MemoryRouter initialEntries={['/rsr-study']} future={{v7_startTransition:true,v7_relativeSplatPath:true}}><Routes>
    <Route element={<StudyAccess />}><Route path="/rsr-study" element={<p>Study controls</p>} /></Route>
    <Route path="/rsr-study/login" element={<p>Researcher sign in</p>} />
  </Routes></MemoryRouter>));
}
it('rejects a forged cached role when the server has no session', async () => {
  mockApi.getStudySession.mockRejectedValue(new Error('HTTP 401'));
  await render(); expect(host.textContent).toContain('Researcher sign in');
  expect(host.textContent).not.toContain('Study controls'); expect(localStorage.getItem('authUser')).toBeNull();
});
it('opens controls only after server verification and replaces stale identity', async () => {
  let resolve;
  mockApi.getStudySession.mockImplementation(() => new Promise(r => { resolve = r; }));
  await render(); expect(host.textContent).not.toContain('Study controls');
  await act(async () => resolve({id:'issued',role:'Researcher',userName:'rsr-examiner'}));
  expect(host.textContent).toContain('Study controls');
  expect(JSON.parse(localStorage.getItem('authUser')).id).toBe('issued');
});
it('keeps study controls closed if verification is unavailable', async () => {
  mockApi.getStudySession.mockRejectedValue(new Error('Network offline'));
  await render(); expect(host.textContent).toContain('Could not verify study access');
  expect(host.textContent).not.toContain('Study controls');
});
