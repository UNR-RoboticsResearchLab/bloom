import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct, Simulate } from 'react-dom/test-utils';
import RsrStudyAdmin from './RsrStudyAdmin';
import { saveChunk, saveSession, readSessions } from './studyStorage';

const mockApi = {
  baseUrl: 'http://localhost:8080',
  getRsrSentenceManifest: jest.fn(), analyzeRsr: jest.fn(),
};
jest.mock('../../context/ApiClientContext', () => ({ useApiClient: () => mockApi }));
jest.mock('../../context/RobotPairingContext', () => ({ useRobotPairing: () => ({ isPaired: false }) }));
jest.mock('./studyStorage', () => ({
  saveSession: jest.fn(() => Promise.resolve()), saveChunk: jest.fn(() => Promise.resolve()),
  readSessions: jest.fn(() => Promise.resolve([])), readAudio: jest.fn(), download: jest.fn(), replaceAudio: jest.fn(),
}));

const act = React.act || legacyAct;
let instances, audioInstances;
const originalAudio = global.Audio;
class Recorder {
  static isTypeSupported() { return true; }
  constructor() { this.state = 'inactive'; this.mimeType = 'audio/webm'; this.listeners = []; instances.push(this); }
  start() { this.state = 'recording'; }
  addEventListener(type, fn) { if (type === 'stop') this.listeners.push(fn); }
  stop() {
    this.state = 'inactive';
    // The last chunk arrives asynchronously with stop, as in a real browser.
    setTimeout(() => {
      this.ondataavailable({ data: new Blob(['tail']) });
      this.onstop(); this.listeners.forEach(fn => fn());
    }, 25);
  }
}
let root, host, stopTrack;
function button(text) { return [...host.querySelectorAll('button')].find(b => b.textContent === text); }
async function click(text) { await act(async () => { Simulate.click(button(text)); }); }
async function setup() {
  const inputs = host.querySelectorAll('input');
  await act(async () => { Simulate.change(inputs[0], { target: {value: 'P-001'} }); Simulate.change(inputs[1], { target: { value: '72' } }); });
  await click('Create session');
}
beforeEach(async () => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  instances = [];
  audioInstances = [];
  global.Audio = jest.fn(src => {
    const audio = { src, play: jest.fn(() => Promise.resolve()), pause: jest.fn() };
    audioInstances.push(audio); return audio;
  });
  global.MediaRecorder = Recorder;
  global.BroadcastChannel = class { postMessage() {} close() {} };
  Object.defineProperty(window, 'crypto', { configurable: true, value: { randomUUID: () => `id-${Math.random()}` } });
  stopTrack = jest.fn();
  const track = { stop: stopTrack };
  Object.defineProperty(navigator, 'mediaDevices', { configurable: true, value: {
    getUserMedia: jest.fn(() => Promise.resolve({ getTracks: () => [track], getAudioTracks: () => [track] })),
  } });
  URL.createObjectURL = jest.fn(() => 'blob:session'); URL.revokeObjectURL = jest.fn();
  mockApi.getRsrSentenceManifest.mockResolvedValue(Array.from({length:16}, (_,i) => ({ id:i+1, text:`Item ${i+1}` })));
  mockApi.analyzeRsr.mockResolvedValue({ sentences: [], totalScore: 0, decision: 'N/A' });
  jest.clearAllMocks();
  saveSession.mockResolvedValue(); saveChunk.mockResolvedValue(); readSessions.mockResolvedValue([]);
  host = document.createElement('div'); document.body.appendChild(host); root = createRoot(host);
  await act(async () => { root.render(<RsrStudyAdmin />); });
});
afterEach(async () => { await act(async () => { root.unmount(); await new Promise(resolve => setTimeout(resolve, 35)); }); host.remove(); global.Audio = originalAudio; });

it('uploads a saved revision only after a click and shows its Box link', async () => {
  const originalFetch = global.fetch;
  global.fetch = jest.fn()
    .mockResolvedValueOnce({ ok: true, json: async () => [{ studyId: 'study', revision: 'revision', metadata: {
      participant: 'TEST', condition: { label: 'Rehearsal' }, createdAt: new Date().toISOString(),
    } }] })
    .mockResolvedValueOnce({ ok: true, json: async () => ({ url: 'https://app.box.com/file/987' }) });
  try {
    await click('Load saved archives');
    expect(global.fetch).toHaveBeenCalledTimes(1);
    await click('Upload to Box');
    expect(global.fetch).toHaveBeenLastCalledWith('http://localhost:8080/api/rsr-study/archives/study/revision/box', {
      method: 'POST', credentials: 'include',
    });
    expect(host.querySelector('a[href="https://app.box.com/file/987"]')).not.toBeNull();
  } finally { global.fetch = originalFetch; }
});

it('requires recording before prompts and keeps recording across prompts, responses, and breaks', async () => {
  await setup();
  expect(button('Cue examiner prompt').disabled).toBe(true);
  await click('Start continuous recording');
  const mr = instances[0];
  // An utterance before any prompt is saved immediately.
  await act(async () => { mr.ondataavailable({ data: new Blob(['early speech']) }); });
  expect(saveChunk).toHaveBeenCalledTimes(1);
  await click('Cue examiner prompt');
  await click('Prompt ended / start response window');
  await click('End attempt');
  await click('Break (keep recording)');
  expect(mr.state).toBe('recording');
  expect(button('Cue examiner prompt').disabled).toBe(true);
  await click('Resume');
  expect(mr.state).toBe('recording');
  expect(instances).toHaveLength(1);
  await click('Finish recording');
  await act(async () => { await new Promise(resolve => setTimeout(resolve, 40)); });
  expect(saveChunk).toHaveBeenCalledTimes(2);
  expect(stopTrack).toHaveBeenCalled();
  expect(host.textContent).toContain('Session finished');
});

it('reports microphone denial and never enables prompt delivery', async () => {
  await setup();
  navigator.mediaDevices.getUserMedia.mockRejectedValue(new Error('Microphone denied'));
  await click('Start continuous recording');
  expect(host.querySelector('[role="alert"]').textContent).toContain('Microphone denied');
  expect(button('Cue examiner prompt').disabled).toBe(true);
  expect(instances).toHaveLength(0);
});

it('releases a late microphone grant when the examiner has left the page', async () => {
  await setup();
  let grant;
  navigator.mediaDevices.getUserMedia.mockImplementation(() => new Promise(resolve => { grant = resolve; }));
  await click('Start continuous recording');
  await act(async () => { root.unmount(); });
  await act(async () => { grant({ getTracks: () => [{stop: stopTrack}] }); });
  expect(stopTrack).toHaveBeenCalledTimes(1);
  expect(instances).toHaveLength(0);
});

it('chimes only after a browser sentence ends and keeps response marking manual', async () => {
  mockApi.getRsrSentenceManifest.mockResolvedValue(Array.from({length:16}, (_,i) => ({id:i+1,text:`Item ${i+1}`,audioUrl:`/rsr-audio/sentence_${i+1}.wav`})));
  await act(async () => { root.unmount(); root = createRoot(host); root.render(<RsrStudyAdmin />); });
  await act(async () => { Simulate.change(host.querySelectorAll('select')[1], {target:{value:'condition-2'}}); });
  await setup(); await click('Start continuous recording'); await click('Play prompt');
  expect(audioInstances).toHaveLength(1);
  expect(audioInstances[0].src).toContain('/rsr-audio/');
  await act(async () => { audioInstances[0].onended(); });
  expect(audioInstances).toHaveLength(2);
  expect(audioInstances[1].src).toBe('/sounds/prompt-complete.wav');
  expect(button('Prompt ended / start response window').disabled).toBe(false);
  expect(saveSession.mock.calls.some(([s]) => s.events.some(e => e.type === 'response_window_started'))).toBe(false);
});

it('does not chime when browser playback fails', async () => {
  mockApi.getRsrSentenceManifest.mockResolvedValue(Array.from({length:16}, (_,i) => ({id:i+1,text:`Item ${i+1}`,audioUrl:`/rsr-audio/sentence_${i+1}.wav`})));
  await act(async () => { root.unmount(); root = createRoot(host); root.render(<RsrStudyAdmin />); });
  await act(async () => { Simulate.change(host.querySelectorAll('select')[1], {target:{value:'condition-2'}}); });
  await setup(); await click('Start continuous recording'); await click('Play prompt');
  await act(async () => { audioInstances[0].onerror(); });
  expect(audioInstances).toHaveLength(1);
  expect(host.textContent).toContain('Playback failed');
});
