import { useState } from 'react';
import { Link } from 'react-router-dom';
import { useApiClient } from '../../context/ApiClientContext';
import './study.css';

export default function RsrResearcherAccounts() {
  const api = useApiClient();
  const [login, setLogin] = useState('');
  const [password, setPassword] = useState('');
  const [fullName, setFullName] = useState('');
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  async function submit(event) {
    event.preventDefault(); if (busy) return;
    setBusy(true); setMessage(''); setError('');
    try {
      const result = await api.createStudyResearcher({ login: login.trim(), password, fullName });
      setPassword(''); setLogin(''); setFullName('');
      setMessage(`Researcher login “${result.login}” is ready. Give the researcher their assigned login and password and the /rsr-study/login address.`);
    } catch(e) { setError(e.message); }
    finally { setBusy(false); }
  }
  return <main className="rsr-study rsr-login"><Link to="/rsr-study">← Study dashboard</Link>
    <h1>Issue a researcher login</h1>
    <p>Only Bloom administrators can create study accounts.</p>
    {message && <p className="rsr-status" role="status">{message}</p>}
    <form className="rsr-card" onSubmit={submit}>
      {error && <p className="rsr-error" role="alert">{error}</p>}
      <label>Researcher name<input value={fullName} maxLength={256} onChange={e => setFullName(e.target.value)} /></label>
      <label>Assigned login<input autoComplete="off" required maxLength={256} value={login} onChange={e => setLogin(e.target.value)} /></label>
      <label>Assigned password<input autoComplete="new-password" type="password" required minLength={6} value={password} onChange={e => setPassword(e.target.value)} /></label>
      <small>Use at least six characters, an uppercase letter, a lowercase letter, and a digit.</small>
      <div className="rsr-actions"><button className="primary" disabled={busy}>{busy ? 'Creating…' : 'Create researcher login'}</button></div>
    </form>
  </main>;
}
