import { useCallback, useEffect, useRef, useState } from 'react';
import { useApiClient } from '../../context/ApiClientContext';
import { useRobotPairing } from '../../context/RobotPairingContext';
import { Link } from 'react-router-dom';
import { getSession, signOut } from '../../utils/auth';
import { PairRobotCard } from '../PairRobotCard';
import { CONDITIONS, responseMarkers, reviewedScores } from './studyModel';
import { saveSession, saveChunk, readSessions, readAudio, replaceAudio, download } from './studyStorage';
import './study.css';

export default function RsrStudyAdmin() {
  const api = useApiClient();
  const { sessionId, isPaired } = useRobotPairing();
  const [conditions, setConditions] = useState(CONDITIONS);
  const [conditionId, setConditionId] = useState(CONDITIONS[0].id);
  const [participant, setParticipant] = useState('');
  const [age, setAge] = useState('');
  const [percentile, setPercentile] = useState('5');
  const [sentences, setSentences] = useState([]);
  const [study, setStudy] = useState(null);
  const [history, setHistory] = useState([]);
  const [archives, setArchives] = useState([]);
  const [lastArchive, setLastArchive] = useState(null);
  const [boxStatus, setBoxStatus] = useState(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [recording, setRecording] = useState(false);
  const [pairedRobot, setPairedRobot] = useState(null);
  const [pairOpen, setPairOpen] = useState(false);
  const [childConnected, setChildConnected] = useState(false);
  const [reason, setReason] = useState('');
  const [overrideScore, setOverrideScore] = useState('');
  const [windowStart, setWindowStart] = useState('');
  const [windowEnd, setWindowEnd] = useState('');
  const [audioUrl, setAudioUrl] = useState('');
  const current = useRef(null);
  const recorder = useRef(null);
  const stream = useRef(null);
  const stopCompletion = useRef(Promise.resolve());
  const player = useRef(null);
  const completionChime = useRef(null);
  const channel = useRef(null);
  const childSeen = useRef(0);
  const clock = useRef(0);
  const writes = useRef(Promise.resolve());
  const chunks = useRef([]);
  const chunkIndex = useRef(0);
  const savingError = useRef(null);
  const operation = useRef(false);
  const mounted = useRef(true);

  const persist = useCallback(value => {
    current.current = value;
    if (mounted.current) setStudy(value);
    writes.current = writes.current.then(() => saveSession(value)).catch(e => {
      savingError.current = e;
      if (mounted.current) setError(`Local storage failed: ${e.message}. Keep this tab open and export the recording.`);
    });
  }, []);
  const elapsed = () => Math.round(performance.now() - clock.current);
  const update = (type, details = {}, changes = {}) => {
    const s = current.current;
    if (!s) return;
    const event = { type, ms: recorder.current?.state === 'recording' ? elapsed() : s.durationMs || 0,
      at: new Date().toISOString(), actor: getSession()?.id, ...details };
    persist({ ...s, ...changes, events: [...s.events, event] });
  };
  const run = async fn => {
    if (operation.current) return;
    operation.current = true; setBusy(true); setError('');
    try { await fn(); } catch(e) { setError(e.message || String(e)); }
    finally { operation.current = false; if (mounted.current) setBusy(false); }
  };

  useEffect(() => {
    api.getRsrSentenceManifest().then(manifest => {
      if (!Array.isArray(manifest) || manifest.length !== 16 || new Set(manifest.map(s => s.id)).size !== 16 || manifest.some(s => !Number.isInteger(s.id) || s.id < 1 || s.id > 16 || !s.text)) throw new Error('The study requires the complete 16-item RSR manifest.');
      setSentences([...manifest].sort((a,b) => a.id-b.id));
    }).catch(e => setError(`Could not load study items: ${e.message}`));
    readSessions().then(list => setHistory(list.filter(s => s.administratorId === getSession()?.id && s.phase !== 'cancelled'))).catch(e => setError(`Local study storage unavailable: ${e.message}`));
  }, [api]);
  useEffect(() => {
    setPairedRobot(null);
    if (!isPaired || !sessionId) return;
    let cancelled = false;
    api.getSessionRobots(sessionId).then(({ robotIds }) => {
      if (!cancelled) setPairedRobot(robotIds?.[0] || null);
    }).catch(e => { if (!cancelled) setError(e.message); });
    return () => { cancelled = true; };
  }, [api, isPaired, sessionId]);
  useEffect(() => {
    if (!study?.id) return;
    childSeen.current = 0;
    const c = new BroadcastChannel(`rsr-child-${study.id}`);
    channel.current = c;
    const publish = () => {
      const s = current.current;
      c.postMessage({ type: 'view', status: s.childStatus, item: s.currentIdx + 1, total: s.sentences.length });
      setChildConnected(Date.now() - childSeen.current < 4000);
    };
    c.onmessage = ({ data }) => { if (data.type === 'ready') { childSeen.current = Date.now(); publish(); } };
    publish();
    const timer = setInterval(publish, 1000);
    return () => { clearInterval(timer); c.close(); channel.current = null; };
  }, [study?.id]);
  useEffect(() => {
    if (!recording) return;
    const warn = e => { e.preventDefault(); e.returnValue = ''; };
    window.addEventListener('beforeunload', warn);
    return () => window.removeEventListener('beforeunload', warn);
  }, [recording]);
  useEffect(() => {
    mounted.current = true;
    return () => {
    mounted.current = false;
    completionChime.current?.pause();
    player.current?.pause();
    if (recorder.current?.state === 'recording') recorder.current.stop();
    stream.current?.getTracks().forEach(t => t.stop());
    };
  }, []);
  useEffect(() => () => { if (audioUrl) URL.revokeObjectURL(audioUrl); }, [audioUrl]);

  async function createStudy() {
    if (!participant.trim() || !Number.isInteger(Number(age)) || Number(age) < 60 || Number(age) > 119)
      throw new Error('Enter a participant code and age from 60 to 119 months.');
    if (!window.BroadcastChannel || !navigator.mediaDevices?.getUserMedia || !window.MediaRecorder || !window.crypto?.randomUUID)
      throw new Error('This browser needs microphone recording and BroadcastChannel support on HTTPS or localhost.');
    if (sentences.length !== 16) throw new Error('Load the complete RSR manifest first.');
    const condition = { ...conditions.find(c => c.id === conditionId) };
    if (conditions.some(c => !c.label.trim())) throw new Error('Give each study condition a label.');
    if (condition.delivery === 'robot' && !pairedRobot) throw new Error('Pair M before creating the robot condition.');
    const s = { schemaVersion: 1, id: crypto.randomUUID(), administratorId: getSession()?.id, createdAt: new Date().toISOString(), participant: participant.trim(),
      ageMonths: Number(age), percentile: Number(percentile), condition, conditions,
      robotId: condition.delivery === 'robot' ? pairedRobot : null, pairingSessionId: sessionId,
      sentences, phase: 'ready', childStatus: 'ready', currentIdx: 0, events: [], attempts: [],
      overrides: {}, results: null, durationMs: 0, notes: '', mimeType: '' };
    // Verify durable storage before allowing recording.
    await saveSession(s); savingError.current = null; chunks.current = []; chunkIndex.current = 0;
    persist(s); setAudioUrl('');
  }

  async function startRecording() {
    const s = current.current;
    if (s.phase !== 'ready') return;
    const media = await navigator.mediaDevices.getUserMedia({ audio: {
      echoCancellation: false, noiseSuppression: false, autoGainControl: false,
    } });
    if (!mounted.current) { media.getTracks().forEach(t => t.stop()); return; }
    stream.current = media;
    try {
      const mimeType = ['audio/webm;codecs=opus', 'audio/ogg;codecs=opus', 'audio/mp4']
        .find(t => MediaRecorder.isTypeSupported(t));
      const mr = new MediaRecorder(media, mimeType ? { mimeType } : undefined);
      recorder.current = mr;
      stopCompletion.current = new Promise(resolve => mr.addEventListener('stop', resolve, { once: true }));
      mr.ondataavailable = ({ data }) => {
        if (!data.size) return;
        chunks.current.push(data);
        const index = chunkIndex.current++;
        if (current.current?.id === s.id && current.current.phase === 'recording')
          persist({ ...current.current, durationMs: elapsed() });
        writes.current = writes.current.then(() => saveChunk(s.id, index, data)).catch(e => {
          savingError.current = e;
          if (mounted.current) setError(`Audio storage failed: ${e.message}. Finish and download before closing.`);
        });
      };
      mr.onerror = () => {
        if (mounted.current) setError('Microphone recording failed. End this session and export the available audio.');
        update('recording_error');
        if (mr.state !== 'inactive') mr.stop();
      };
      mr.onstop = () => {
        media.getTracks().forEach(t => t.stop());
        const latest = current.current;
        if (latest?.phase === 'recording') {
          persist({ ...latest, phase: 'interrupted', childStatus: 'paused', durationMs: elapsed(),
            events: [...latest.events, { type: 'recording_interrupted', ms: elapsed(), at: new Date().toISOString() }] });
        }
        if (mounted.current) setRecording(false);
      };
      media.getAudioTracks().forEach(track => track.onended = () => {
        if (mr.state === 'recording') mr.stop();
      });
      clock.current = performance.now(); mr.start(1000); setRecording(true);
      persist({ ...s, phase: 'recording', mimeType: mr.mimeType,
        events: [{ type: 'recording_started', ms: 0, at: new Date().toISOString() }] });
    } catch(e) { media.getTracks().forEach(t => t.stop()); throw e; }
  }

  const activeAttempt = () => current.current.attempts.find(a => a.end == null);
  function notifyPromptFinished(attemptId) {
    if (!mounted.current || !current.current?.attempts.some(a => a.id === attemptId && a.end == null)) return;
    completionChime.current?.pause();
    const cue = new Audio('/sounds/prompt-complete.wav');
    completionChime.current = cue;
    cue.volume = 0.5;
    cue.onended = () => { if (completionChime.current === cue) completionChime.current = null; };
    Promise.resolve(cue.play()).then(() => {
      if (mounted.current && current.current?.attempts.some(a => a.id === attemptId))
        update('prompt_completion_chime', { attemptId });
    }).catch(() => {
      if (mounted.current) setError('The sentence finished, but its notification chime could not play. Check browser audio permissions.');
    });
  }
  async function playPrompt() {
    const s = current.current;
    if (recorder.current?.state !== 'recording' || s.childStatus === 'paused')
      throw new Error('Start continuous recording and resume before presenting an item.');
    if (activeAttempt()) throw new Error('End the current attempt before replaying or moving on.');
    const sentence = s.sentences[s.currentIdx];
    const attempt = { id: crypto.randomUUID(), sentenceId: sentence.id, start: elapsed(), responseStart: null, end: null };
    update('prompt_requested', { sentenceId: sentence.id, attemptId: attempt.id, delivery: s.condition.delivery },
      { attempts: [...s.attempts, attempt], childStatus: 'listening' });
    try {
      if (s.condition.delivery === 'robot') {
        if (pairedRobot !== s.robotId) throw new Error('The study’s paired robot changed. Restore its pairing before continuing.');
        const command = await api.queueRsrSentence(s.robotId, sentence.id);
        update('robot_prompt_queued', { sentenceId: sentence.id, commandId: command.commandId });
      } else if (s.condition.delivery === 'browser') {
        if (!sentence.audioUrl) throw new Error('No stimulus audio exists for this item.');
        const audio = new Audio(new URL(sentence.audioUrl, `${api.baseUrl.replace(/\/$/, '')}/`).href);
        player.current = audio;
        audio.onplaying = () => update('browser_playback_started', { attemptId: attempt.id });
        audio.onended = () => { update('browser_playback_ended', { attemptId: attempt.id }); player.current = null; notifyPromptFinished(attempt.id); };
        audio.onerror = () => { player.current = null; update('playback_error', { attemptId: attempt.id }); setError('Playback failed. End this attempt, exclude it with a reason, then replay.'); };
        await audio.play();
      } else update('examiner_prompt_cued', { attemptId: attempt.id });
    } catch(e) {
      update('prompt_failed', { attemptId: attempt.id, message: e.message }, {
        attempts: current.current.attempts.map(a => a.id === attempt.id ? { ...a, end: elapsed(), excluded: true, reason: e.message } : a),
        childStatus: 'ready',
      });
      throw e;
    }
  }
  function markResponse() {
    const a = activeAttempt();
    if (!a || a.responseStart != null || player.current) return;
    if (current.current.condition.delivery === 'examiner') notifyPromptFinished(a.id);
    update('response_window_started', { attemptId: a.id, source: 'administrator' }, {
      attempts: current.current.attempts.map(t => t.id === a.id ? { ...t, responseStart: elapsed() } : t), childStatus: 'responding',
    });
  }
  function endAttempt() {
    const a = activeAttempt();
    if (!a) return;
    player.current?.pause(); player.current = null;
    update('attempt_ended', { attemptId: a.id }, {
      attempts: current.current.attempts.map(t => t.id === a.id ? { ...t, end: elapsed() } : t), childStatus: 'ready',
    });
  }
  async function finish() {
    if (activeAttempt()) endAttempt();
    player.current?.pause(); player.current = null;
    const mr = recorder.current;
    update('session_finished', {}, { phase: 'finished', childStatus: 'finished', durationMs: elapsed() });
    if (mr?.state !== 'inactive') mr.stop();
    await stopCompletion.current;
    await writes.current;
    const blob = new Blob(chunks.current, { type: current.current.mimeType });
    setAudioUrl(URL.createObjectURL(blob));
    if (savingError.current) throw new Error('Some local saves failed. Download audio and the session log now.');
  }
  async function audioBlob() {
    if (chunks.current.length) return new Blob(chunks.current, { type: current.current.mimeType });
    return readAudio(current.current.id, current.current.mimeType);
  }
  async function analyze() {
    const s = current.current;
    const markers = responseMarkers(s.attempts);
    if (!markers.length) throw new Error('Mark at least one response window before analysis.');
    const blob = await audioBlob();
    if (!blob.size) throw new Error('No session audio is available.');
    const form = new FormData();
    form.append('sessionAudio', blob, `session.${extension(s.mimeType)}`);
    form.append('age', s.ageMonths); form.append('percentile', s.percentile);
    form.append('markers', JSON.stringify(markers));
    const results = await api.analyzeRsr(form);
    update('automatic_analysis_completed', { assessmentId: results.assessmentId }, { results: { ...results, analyzedSentenceIds: markers.map(m => m.sentence_id) } });
  }
  async function archive() {
    const s = current.current;
    const form = new FormData();
    form.append('metadata', JSON.stringify({ ...s, reviewedScores: reviewedScores(s.sentences, s.results, s.overrides) }));
    const blob = await audioBlob();
    if (!blob.size) throw new Error('No recording is available to archive.');
    form.append('audio', blob, `session.${extension(s.mimeType)}`);
    const response = await fetch(`${api.baseUrl}/api/rsr-study/archives/${s.id}`, { method: 'POST', credentials: 'include', body: form });
    if (!response.ok) throw new Error(`Archive failed (${response.status}). Your local recording is retained; retry or download it.`);
    const result = await response.json();
    setLastArchive({ studyId: s.id, revision: result.revision });
    setBoxStatus(null);
    update('archive_saved', { revision: result.revision });
  }
  async function uploadToBox(saved) {
    setBoxStatus(null);
    const response = await fetch(`${api.baseUrl}/api/rsr-study/archives/${saved.studyId}/${saved.revision}/box`, {
      method: 'POST', credentials: 'include',
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || 'Box upload failed. Your Bloom archive is retained; you can retry.');
    setBoxStatus(result);
  }
  async function openSaved(s) {
    chunks.current = []; savingError.current = null;
    const recovered = ['recording', 'ready'].includes(s.phase) ? { ...s, phase: 'interrupted', childStatus: 'paused' } : s;
    persist(recovered);
    const blob = await readAudio(s.id, s.mimeType);
    setAudioUrl(blob.size ? URL.createObjectURL(blob) : '');
  }
  const sentence = study?.sentences[study.currentIdx];
  const active = study?.attempts.find(a => a.end == null);
  const scores = study ? reviewedScores(study.sentences, study.results, study.overrides) : [];
  const reviewed = scores.filter(s => s.reviewed != null);
  const stopped = study && ['finished', 'interrupted'].includes(study.phase);

  return <main className="rsr-study">
    <h1>RSR study control room</h1>
    <div className="rsr-actions"><a className="rsr-link" href="https://unr-roboticsresearchlab.github.io/bloom/docs/training/index.html" target="_blank" rel="noopener noreferrer">Researcher training</a><span>Researcher: {getSession()?.fullName || getSession()?.userName}</span>
      {getSession()?.role?.toLowerCase() === 'admin' && <Link className="rsr-link" to="/rsr-study/accounts">Issue researcher login</Link>}
      <button disabled={busy || recording || study?.phase === 'recording'} onClick={() => run(async () => {
        await writes.current; await api.studySignOut(); signOut(); window.location.assign('/rsr-study/login');
      })}>Sign out of study</button>
    </div>
    <p>Redmond Sentence Recall · Examiner controls and review</p>
    {error && <div className="rsr-error" role="alert">{error}</div>}
    {boxStatus && <p role="status">Recording uploaded to Box. <a href={boxStatus.url} target="_blank" rel="noreferrer">Open uploaded recording</a></p>}
    {!study ? <>
      <section className="rsr-card">
        <h2>Set up a session</h2>
        <p>Condition labels and delivery are provisional. Set these to your approved study protocol.</p>
        <div className="rsr-grid"><div>
          <label>Participant code<input value={participant} onChange={e => setParticipant(e.target.value)} /></label>
          <label>Age in months (60–119)<input type="number" min="60" max="119" value={age} onChange={e => setAge(e.target.value)} /></label>
          <label>Percentile cutoff<select value={percentile} onChange={e => setPercentile(e.target.value)}>{[5,10,15].map(p => <option key={p}>{p}</option>)}</select></label>
          <label>Assigned condition<select value={conditionId} onChange={e => setConditionId(e.target.value)}>{conditions.map(c => <option key={c.id} value={c.id}>{c.label}</option>)}</select></label>
        </div><div>
          {conditions.map(c => <div key={c.id}>
            <label>{c.id} label<input value={c.label} onChange={e => setConditions(list => list.map(t => t.id === c.id ? { ...t, label: e.target.value } : t))} /></label>
            <label>Delivery<select value={c.delivery} onChange={e => setConditions(list => list.map(t => t.id === c.id ? { ...t, delivery: e.target.value } : t))}>
              <option value="examiner">Examiner reads aloud</option><option value="browser">Browser stimulus audio</option><option value="robot">M robot</option>
            </select></label>
          </div>)}
        </div></div>
        <div className="rsr-actions"><button onClick={() => setPairOpen(true)}>Pair M robot</button><span>{pairedRobot ? 'Robot paired' : 'No robot paired'}</span>
          <button className="primary" disabled={busy || sentences.length !== 16} onClick={() => run(createStudy)}>Create session</button></div>
      </section>
      <section className="rsr-card"><h2>Sessions saved on this browser</h2>
        {history.length === 0 && <p>No saved sessions.</p>}
        {history.map(s => <div key={s.id} className="rsr-actions"><span>{s.participant} · {s.condition.label} · {new Date(s.createdAt).toLocaleString()} · {s.phase}</span><button disabled={busy} onClick={() => run(() => openSaved(s))}>Review / recover</button></div>)}
      </section>
      <section className="rsr-card"><h2>Your archives on Bloom</h2>
        <button disabled={busy} onClick={() => run(async () => {
          const response = await fetch(`${api.baseUrl}/api/rsr-study/archives`, { credentials: 'include' });
          if (!response.ok) throw new Error(`Could not load archives (${response.status}).`);
          setArchives(await response.json());
        })}>Load saved archives</button>
        {archives.map(a => <div key={a.studyId} className="rsr-actions"><span>{a.metadata.participant} · {a.metadata.condition.label} · {new Date(a.metadata.createdAt).toLocaleString()}</span>
          <button disabled={busy} onClick={() => run(async () => {
            const response = await fetch(`${api.baseUrl}/api/rsr-study/archives/${a.studyId}/${a.revision}/audio`, { credentials: 'include' });
            if (!response.ok) throw new Error(`Could not load recording (${response.status}).`);
            const blob = await response.blob();
            await writes.current;
            await replaceAudio(a.studyId, blob);
            chunks.current = [blob]; savingError.current = null; persist(a.metadata);
            setAudioUrl(URL.createObjectURL(blob));
          })}>Open archive</button>
          <button disabled={busy} onClick={() => run(() => uploadToBox(a))}>Upload to Box</button>
        </div>)}
      </section>
    </> : <>
      <div className="rsr-status" role="status">{recording ? '● Continuous recording is running' : `Session ${study.phase}`} · {study.participant} · {study.condition.label} · {study.condition.delivery}
        <br /><small>{childConnected ? 'Child display connected' : 'Child display disconnected'} · Audio stays on across prompts, responses, breaks, and item changes.</small></div>
      <div className="rsr-actions"><button disabled={busy} onClick={() => window.open(`/rsr-study/child/${study.id}`, `rsr-child-${study.id}`)}>Open child display</button>
        {study.phase === 'ready' && <><button className="primary" disabled={busy} onClick={() => run(startRecording)}>Start continuous recording</button><button disabled={busy} onClick={() => { update('setup_cancelled', {}, {phase: 'cancelled'}); current.current = null; setStudy(null); }}>Cancel setup</button></>}
        {recording && <><button disabled={busy || !!active} onClick={() => update(study.childStatus === 'paused' ? 'resumed' : 'break_started', {}, { childStatus: study.childStatus === 'paused' ? 'ready' : 'paused' })}>{study.childStatus === 'paused' ? 'Resume' : 'Break (keep recording)'}</button>
          <button className="danger" disabled={busy} onClick={() => run(finish)}>Finish recording</button></>}
      </div>
      <div className="rsr-grid"><section className="rsr-card"><h2>Item {study.currentIdx + 1} / {study.sentences.length}</h2>
        <p className="rsr-stimulus">{sentence?.text}</p>
        <div className="rsr-actions"><button className="primary" disabled={busy || !recording || !!active || study.childStatus === 'paused'} onClick={() => run(playPrompt)}>{study.condition.delivery === 'examiner' ? 'Cue examiner prompt' : 'Play prompt'}</button>
          <button disabled={busy || !recording || !active || active.responseStart != null || !!player.current} onClick={markResponse}>Prompt ended / start response window</button>
          <button disabled={busy || !recording || !active} onClick={endAttempt}>End attempt</button></div>
        <small>{study.condition.delivery === 'examiner' ? 'Click Prompt ended when you finish reading; a short chime signals the response cue.' : 'A short chime signals the end of the sentence. Mark the response window after the chime.'} Early speech remains in the full recording.</small>
        <div className="rsr-items" style={{ marginTop: 20 }}>{study.sentences.map((s,i) => <button key={s.id} disabled={busy || !!active} aria-current={i === study.currentIdx} onClick={() => update('item_selected', { sentenceId: s.id }, { currentIdx: i })}>{s.id}</button>)}</div>
        <label>Reason for override or exclusion<textarea value={reason} onChange={e => setReason(e.target.value)} /></label>
        <div className="rsr-actions"><label>Reviewed score<select value={overrideScore} onChange={e => setOverrideScore(e.target.value)}><option value="">Select</option>{[0,1,2].map(n => <option key={n}>{n}</option>)}</select></label>
          <button disabled={busy || !reason.trim() || overrideScore === ''} onClick={() => {
            update('score_overridden', { sentenceId: sentence.id, previous: scores.find(s => s.id === sentence.id)?.reviewed, score: Number(overrideScore), reason: reason.trim() },
              { overrides: { ...study.overrides, [sentence.id]: { score: Number(overrideScore), reason: reason.trim() } } }); setReason(''); setOverrideScore('');
          }}>Apply score override</button></div>
        {stopped && <div>
          <h2>Correct response window</h2>
          <small>Times in seconds from the start of the full recording. Enter a reason above. This updates the latest eligible attempt for this item.</small>
          <div className="rsr-actions"><label>Start (seconds)<input type="number" min="0" step="0.1" value={windowStart} onChange={e => setWindowStart(e.target.value)} /></label>
            <label>End (seconds)<input type="number" min="0" step="0.1" value={windowEnd} onChange={e => setWindowEnd(e.target.value)} /></label>
            <button disabled={busy || !!study.results || !reason.trim() || windowStart === '' || windowEnd === ''} onClick={() => run(async () => {
              const start = Math.round(Number(windowStart)*1000), end = Math.round(Number(windowEnd)*1000);
              if (!Number.isFinite(start) || !Number.isFinite(end) || start < 0 || end <= start || end > study.durationMs)
                throw new Error('The response window must be within the recorded session, with end after start.');
              const attempt = study.attempts.filter(a => a.sentenceId === sentence.id && !a.excluded).slice(-1)[0];
              if (!attempt) throw new Error('This item has no eligible attempt.');
              update('response_window_corrected', { attemptId: attempt.id, previous: {start: attempt.responseStart, end: attempt.end}, start, end, reason: reason.trim() }, {
                attempts: study.attempts.map(a => a.id === attempt.id ? {...a, responseStart: start, end} : a),
              }); setReason(''); setWindowStart(''); setWindowEnd('');
            })}>Save corrected window</button></div>
        </div>}
        <h2>Attempts for this item</h2>
        {study.attempts.filter(a => a.sentenceId === sentence.id).map((a,i) => <div key={a.id} className="rsr-actions">
          <span>#{i+1} · {(a.start / 1000).toFixed(1)}s · {a.end == null ? 'open' : a.excluded ? 'excluded' : a.responseStart == null ? 'no response window' : 'marked'}</span>
          <button disabled={busy || !!study.results || a.end == null || a.excluded || !reason.trim()} onClick={() => {
            update('attempt_excluded', { attemptId: a.id, reason: reason.trim() }, { attempts: study.attempts.map(t => t.id === a.id ? { ...t, excluded: true, reason: reason.trim() } : t) }); setReason('');
          }}>Exclude with reason</button>
        </div>)}
      </section><aside>
        <section className="rsr-card"><h2>Review</h2><p style={{ fontSize: '2rem' }}>{reviewed.reduce((total,s) => total+s.reviewed, 0)} / 32</p><p>{reviewed.length} of 16 items scored</p>
          <small>Unscored items are not zeroes. Overrides do not change the automated screening decision.</small>
          <label>Session notes<textarea value={study.notes} onChange={e => update('notes_updated', {}, { notes: e.target.value })} /></label>
          {stopped && <><div className="rsr-actions"><button disabled={busy || !!study.results} onClick={() => run(analyze)}>{study.results ? 'Automatic scoring complete' : 'Analyze response windows'}</button>
            <button disabled={busy} onClick={() => run(archive)}>Save archive to Bloom</button>
            <button disabled={busy || lastArchive?.studyId !== study.id} onClick={() => run(() => uploadToBox(lastArchive))}>Upload saved recording to Box</button></div>
            <small>Save an archive first. Box uploads copy that saved revision; save again to include a newer recording.</small>
            {audioUrl && <audio controls src={audioUrl} style={{ width: '100%' }} />}
            <div className="rsr-actions"><button disabled={busy} onClick={() => run(async () => download(await audioBlob(), `${study.id}.${extension(study.mimeType)}`))}>Download full audio</button>
              <button disabled={busy} onClick={() => download(new Blob([JSON.stringify({ ...study, reviewedScores: scores }, null, 2)], { type: 'application/json' }), `${study.id}.json`)}>Download session log</button></div>
          </>}
        </section>
        <section className="rsr-card"><h2>Event timeline</h2><ol className="rsr-events">{study.events.slice().reverse().map((e,i) => <li key={i}>{(e.ms/1000).toFixed(1)}s · {e.type.replaceAll('_',' ')}</li>)}</ol></section>
      </aside></div>
      {study.results && <section className="rsr-card"><h2>Automatic score: {study.results.totalScore} / 32 · {study.results.decision}</h2><p>{study.results.analyzedSentenceIds?.length ?? 16} of 16 items submitted for analysis. Review completeness before interpreting this result.</p>
        <table><thead><tr><th>Item</th><th>Transcript</th><th>Automatic</th><th>Reviewed</th><th>Override reason</th></tr></thead><tbody>{scores.map(s => <tr key={s.id}><td>{s.id}</td><td>{study.results.sentences.find(r => String(r.id) === String(s.id))?.response || '—'}</td><td>{s.automatic ?? '—'}</td><td>{s.reviewed ?? '—'}</td><td>{s.reason}</td></tr>)}</tbody></table>
      </section>}
      {stopped && <button disabled={busy} onClick={() => run(async () => { await writes.current; setHistory((await readSessions()).filter(s => s.administratorId === getSession()?.id && s.phase !== 'cancelled')); current.current = null; setStudy(null); setAudioUrl(''); })}>Return to study setup</button>}
    </>}
    {pairOpen && <PairRobotCard onCancel={() => setPairOpen(false)} onPaired={() => setPairOpen(false)} />}
  </main>;
}
function extension(mimeType = '') { return mimeType.includes('ogg') ? 'ogg' : mimeType.includes('mp4') ? 'm4a' : 'webm'; }
