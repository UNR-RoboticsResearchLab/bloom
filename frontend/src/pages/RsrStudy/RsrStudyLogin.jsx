import { useState } from 'react';
import { useApiClient } from '../../context/ApiClientContext';
import { signInSession } from '../../utils/auth';
import './study.css';

export default function RsrStudyLogin() {
  const api = useApiClient();
  const [login, setLogin] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  async function submit(event) {
    event.preventDefault();
    if (busy) return;
    setBusy(true); setError('');
    try {
      const data = await api.studySignIn(login.trim(), password);
      signInSession(data.user);
      window.location.assign('/rsr-study');
    } catch(e) {
      setError(/HTTP (400|401|403)/.test(e.message)
        ? 'The researcher login or password is incorrect. Use the credentials provided by your study administrator.'
        : 'Could not reach the study server. Please try again.');
    } finally { setBusy(false); }
  }
  return <main className="rsr-study rsr-login">
    <p>Bloom · Redmond Sentence Recall</p>
    <h1>Researcher sign in</h1>
    <p>Use the login and password provided by your study administrator.</p>
    <form className="rsr-card" onSubmit={submit}>
      {error && <div className="rsr-error" role="alert">{error}</div>}
      <label htmlFor="researcher-login">Researcher login</label>
      <input id="researcher-login" name="username" autoComplete="username" required maxLength={256} value={login} onChange={e => setLogin(e.target.value)} />
      <label htmlFor="researcher-password">Password</label>
      <input id="researcher-password" name="password" type="password" autoComplete="current-password" required value={password} onChange={e => setPassword(e.target.value)} />
      <div className="rsr-actions"><button className="primary" disabled={busy || !login.trim() || !password}>{busy ? 'Signing in…' : 'Sign in to study'}</button></div>
    </form>
    <small>Need access or a password reset? Contact your study administrator.</small>
  </main>;
}
