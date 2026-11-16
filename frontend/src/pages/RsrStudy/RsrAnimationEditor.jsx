import { useEffect, useRef, useState } from 'react';
import { Link, useOutletContext } from 'react-router-dom';
import { useApiClient } from '../../context/ApiClientContext';
import './study.css';

const endpoint = '/api/rsr-study/animation-behaviors';

export default function RsrAnimationEditor() {
  const api = useApiClient();
  const { user } = useOutletContext();
  const canSave = user.role?.toLowerCase() === 'admin';
  const frame = useRef(null);
  const saving = useRef(false);
  const [saved, setSaved] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    if (!canSave) return;
    let active = true;
    api.request(endpoint).then(rows => {
      if (active) setSaved(rows);
    }).catch(() => {
      if (active) setError('Could not load saved behaviors.');
    });
    return () => { active = false; };
  }, [api, canSave]);

  useEffect(() => {
    if (!canSave) return;
    async function save(event) {
      if (event.origin !== window.location.origin || event.source !== frame.current?.contentWindow ||
          event.data?.type !== 'bloom:save-behavior' || saving.current) return;
      const reply = (ok, message) => event.source.postMessage(
        { type: 'bloom:animator-saved', ok, message }, event.origin);
      saving.current = true;
      try {
        const row = await api.request(endpoint, {
          method: 'POST', body: JSON.stringify(event.data.payload),
        });
        setSaved(rows => [row, ...rows].slice(0, 100));
        reply(true, 'Behavior saved to Bloom.');
      } catch (err) {
        reply(false, /HTTP (401|403)/.test(err.message)
          ? 'Sign in again as a study administrator to save.'
          : 'Could not save. Check the behavior name and timeline, or export JSON to keep a copy.');
      } finally {
        saving.current = false;
      }
    }
    window.addEventListener('message', save);
    return () => window.removeEventListener('message', save);
  }, [api, canSave]);

  return <main className="rsr-study">
    <Link to="/rsr-study">← Study dashboard</Link>
    <h1>M Keyframe Animator</h1>
    <p>Connect to M from this computer. With the lab SSH tunnel running, use <code>http://127.0.0.1:5500</code> as the Robot URL.</p>
    <p>{canSave ? 'Save to Bloom stores each version separately in the study database.' : 'Export JSON keeps a local copy. Only study administrators can save to Bloom.'} Saving does not move M or publish to its static behavior menu.</p>
    <iframe ref={frame} title="M Keyframe Animator" src="/m-keyframe-editor/index.html?embed=bloom"
      style={{ width: '100%', height: '80vh', minHeight: 620, border: 0 }}
      onLoad={() => frame.current?.contentWindow?.postMessage(
        { type: 'bloom:animator-config', canSave }, window.location.origin)} />
    {canSave && <section>
      <h2>Saved behaviors</h2>
      {error && <p role="alert">{error}</p>}
      {saved.length ? <ul>{saved.map(row => <li key={row.id}>
        <a href={`${api.baseUrl}${endpoint}/${row.id}`} download>{row.name}</a>
        {' · '}{new Date(row.createdAtUtc).toLocaleString()}
      </li>)}</ul> : !error && <p>No saved behaviors yet.</p>}
      <p>Download a saved behavior and use Import JSON to edit it in the animator.</p>
    </section>}
  </main>;
}
