import { useEffect, useState } from 'react';
import { useParams } from 'react-router-dom';
import './study.css';

// Child messages deliberately contain no sentence text, scores, or participant data.
export default function RsrStudyChild() {
  const { studyId } = useParams();
  const [view, setView] = useState(null);
  const [connected, setConnected] = useState(false);
  useEffect(() => {
    const channel = new BroadcastChannel(`rsr-child-${studyId}`);
    let last = 0;
    channel.onmessage = ({ data }) => {
      if (data.type !== 'view') return;
      last = Date.now(); setView(data); setConnected(true);
    };
    channel.postMessage({ type: 'ready' });
    const timer = setInterval(() => {
      setConnected(Date.now() - last < 5000);
      channel.postMessage({ type: 'ready' });
    }, 1000);
    return () => { clearInterval(timer); channel.close(); };
  }, [studyId]);
  const messages = { ready: 'Let’s get ready', listening: 'Listen carefully',
    responding: 'Your turn', paused: 'We’re taking a break', finished: 'Thank you!' };
  return <main className="rsr-child" aria-live="polite">
    <div className="rsr-orb" aria-hidden="true">{view?.status === 'finished' ? '★' : '✦'}</div>
    <h1>{connected ? messages[view?.status] || messages.ready : 'Waiting for your examiner'}</h1>
    {connected && view?.status !== 'finished' && view?.item > 0 && <p>Sentence {view.item} of {view.total}</p>}
    {connected && view?.status === 'responding' && <p>Say the sentence you heard.</p>}
  </main>;
}
