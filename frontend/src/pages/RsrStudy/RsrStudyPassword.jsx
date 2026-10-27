import { useState } from 'react';
import { useApiClient } from '../../context/ApiClientContext';
import { signInSession } from '../../utils/auth';
import './study.css';

export default function RsrStudyPassword() {
  const api = useApiClient();
  const [current, setCurrent] = useState('');
  const [password, setPassword] = useState('');
  const [confirm, setConfirm] = useState('');
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  async function submit(event) {
    event.preventDefault();
    if (busy) return;
    if (password !== confirm) { setError('The new passwords do not match.'); return; }
    setBusy(true); setError('');
    try {
      const result = await api.changeStudyPassword(current, password);
      signInSession(result.user);
      window.location.assign('/rsr-study');
    } catch (e) { setError(e.message); }
    finally { setBusy(false); }
  }
  return <main className="rsr-study rsr-login">
    <h1>Set your personal password</h1>
    <p>Replace your temporary password before opening the study.</p>
    <form className="rsr-card" onSubmit={submit}>
      {error && <p role="alert" className="rsr-error">{error}</p>}
      <label>Temporary or current password<input type="password" autoComplete="current-password" required value={current} onChange={e => setCurrent(e.target.value)} /></label>
      <label>New password<input type="password" autoComplete="new-password" required minLength={12} value={password} onChange={e => setPassword(e.target.value)} /></label>
      <label>Confirm new password<input type="password" autoComplete="new-password" required minLength={12} value={confirm} onChange={e => setConfirm(e.target.value)} /></label>
      <p>Use at least 12 characters with uppercase and lowercase letters, a number, and a symbol.</p>
      <button className="primary" disabled={busy}>{busy ? 'Saving…' : 'Set password and open study'}</button>
    </form>
  </main>;
}
