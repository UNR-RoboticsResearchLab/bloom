import { useState } from 'react';
import { Link } from 'react-router-dom';
import { useApiClient } from '../../context/ApiClientContext';
import './study.css';

export default function RsrResearcherAccounts() {
  const api = useApiClient();
  const [login, setLogin] = useState('');
  const [temporaryPassword, setTemporaryPassword] = useState('');
  const [fullName, setFullName] = useState('');
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  async function submit(event) {
    event.preventDefault(); if (busy) return;
    setBusy(true); setMessage(''); setError(''); setTemporaryPassword('');
    try {
      const result = await api.createStudyResearcher({ login: login.trim(), fullName });
      setTemporaryPassword(result.temporaryPassword); setLogin(''); setFullName('');
      setMessage(`Researcher login “${result.login}” is ready. Give the researcher their assigned login and password and the /rsr-study/login address.`);
    } catch(e) { setError(e.message); }
    finally { setBusy(false); }
  }
  return <main className="rsr-study rsr-login"><Link to="/rsr-study">← Study dashboard</Link>
    <h1>Issue a researcher login</h1>
    <p>Only Bloom administrators can create study accounts.</p>
    {message && <div className="rsr-status" role="status"><p>{message}</p><p>Temporary password: <code>{temporaryPassword}</code></p><p>Save it now. The researcher must choose a personal password on first sign in.</p></div>}
    <p>A temporary password is generated automatically for each new researcher.</p>
    <form className="rsr-card" onSubmit={submit}>
      {error && <p className="rsr-error" role="alert">{error}</p>}
      <label>Researcher name<input value={fullName} maxLength={256} onChange={e => setFullName(e.target.value)} /></label>
      <label>Assigned login<input autoComplete="off" required maxLength={256} value={login} onChange={e => setLogin(e.target.value)} /></label>
      <div className="rsr-actions"><button className="primary" disabled={busy}>{busy ? 'Creating…' : 'Create researcher login'}</button></div>
    </form>
  </main>;
}
