import { useEffect, useState } from 'react';
import { Navigate, Outlet } from 'react-router-dom';
import { useApiClient } from '../../context/ApiClientContext';
import { signInSession, signOut } from '../../utils/auth';

export default function StudyAccess({ adminOnly = false, passwordSetup = false }) {
  const api = useApiClient();
  const [access, setAccess] = useState('checking');
  const [attempt, setAttempt] = useState(0);
  useEffect(() => {
    let cancelled = false;
    setAccess('checking');
    // A cached localStorage role is not proof of an authenticated study session.
    api.getStudySession().then(user => {
      if (cancelled) return;
      if (!user?.id || !['researcher', 'admin'].includes(user.role?.toLowerCase())) {
        signOut(); setAccess('login'); return;
      }
      signInSession(user);
      if (user.mustChangePassword && !passwordSetup) { setAccess("password"); return; }
      setAccess(adminOnly && user.role?.toLowerCase() !== 'admin' ? 'researcher' : 'allowed');
    }).catch(error => {
      if (cancelled) return;
      if (/HTTP (401|403)/.test(error.message)) { signOut(); setAccess('login'); }
      else setAccess('error');
    });
    return () => { cancelled = true; };
  }, [api, adminOnly, passwordSetup, attempt]);
  if (access === "password") return <Navigate to="/rsr-study/password" replace />;
  if (access === 'login') return <Navigate to="/rsr-study/login" replace />;
  if (access === 'researcher') return <Navigate to="/rsr-study" replace />;
  if (access === 'checking') return <p role="status">Checking researcher access…</p>;
  if (access === 'error') return <div role="alert"><p>Could not verify study access. Check the connection and try again.</p><button onClick={() => setAttempt(n => n+1)}>Retry</button></div>;
  return <Outlet />;
}
