import { useState, useRef, useEffect } from "react";
import { useApiClient } from "../context/ApiClientContext";
import { useRobotPairing } from "../context/RobotPairingContext";
import { PairRobotCard } from "./PairRobotCard";

// Fallback sentence text, used only if the backend manifest (with pre-generated
// audio URLs) can't be fetched — keeps the page usable for the no-audio flow.
const SENTENCES = [
  "The big football player washed the car with the hose.",
  "All of the pictures were colored by his little sister.",
  "The rose bushes were planted yesterday by the girl scouts.",
  "The happy little girl kicked the ball over the fence.",
  "His little brother cleaned the dirty dishes and cups.",
  "A special cage was made to hold the dangerous animals.",
  "Everybody in my school colored Easter eggs for the picnic.",
  "A new hole was dug for the kid's swimming pool.",
  "Only the first graders made a birdhouse for their parents.",
  "My little sister's dog caught the ball on the first bounce.",
  "The soccer ball was kicked into the school's parking lot.",
  "The lion's teeth were cleaned with a giant toothbrush.",
  "Some of the kids dug holes in the sand two feet deep.",
  "The little white mouse was caught by our neighbor's cat.",
  "The second grade students planted coconuts in the garden.",
  "The dirty clothes were washed with soap one more time.",
];

const PHASE = { SETUP: "setup", ASSESSMENT: "assessment", SUBMITTING: "submitting", RESULTS: "results" };

export default function RsrAssessment() {
  const api = useApiClient();
  const { isPaired, sessionId } = useRobotPairing();

  const [phase, setPhase] = useState(PHASE.SETUP);
  const [ageMonths, setAgeMonths] = useState("");
  const [percentile, setPercentile] = useState("5");

  const [currentIdx, setCurrentIdx] = useState(0);
  const [markers, setMarkers] = useState([]);
  const [isRecording, setIsRecording] = useState(false);
  const [recordingStart, setRecordingStart] = useState(null);

  const [results, setResults] = useState(null);
  const [error, setError] = useState(null);

  // Sentence playback: manifest (with pre-generated audio) fetched on mount,
  // falling back to the hardcoded text-only list if the backend isn't reachable.
  const [sentences, setSentences] = useState(
    SENTENCES.map((text, i) => ({ id: i + 1, text, audioUrl: null, visemeUrl: null }))
  );
  const [playMode, setPlayMode] = useState("browser"); // "browser" | "robot"
  const [showPairCard, setShowPairCard] = useState(false);
  const [robotId, setRobotId] = useState(null);
  const [pairError, setPairError] = useState(null);
  const [playError, setPlayError] = useState(null);
  const [sentToRobotId, setSentToRobotId] = useState(null);

  const mediaRecorderRef = useRef(null);
  const chunksRef = useRef([]);
  const sessionStartRef = useRef(null);
  const streamRef = useRef(null);
  const audioPlayerRef = useRef(null);

  useEffect(() => {
    api.getRsrSentenceManifest()
      .then((manifest) => {
        if (Array.isArray(manifest) && manifest.length > 0) {
          setSentences(manifest);
        }
      })
      .catch(() => {
        // Keep the hardcoded fallback; browser playback just won't be available.
      });
  }, [api]);

  function preferredMimeType() {
    const types = ["audio/webm;codecs=opus", "audio/webm", "audio/ogg;codecs=opus"];
    return types.find((t) => MediaRecorder.isTypeSupported(t)) ?? "";
  }

  // Mirror how lessons resolve a robot: pairing (RobotPairingContext) claims a
  // session, then the robot actually attached to that session is looked up
  // from the session's robot membership.
  useEffect(() => {
    if (!isPaired || !sessionId) {
      setRobotId(null);
      return;
    }
    let cancelled = false;
    api.getSessionRobots(sessionId)
      .then(({ robotIds }) => {
        if (cancelled) return;
        if (!robotIds || robotIds.length === 0) {
          setRobotId(null);
          setPairError("No robot is paired with that session.");
          return;
        }
        setRobotId(robotIds[0]);
        setPairError(null);
      })
      .catch((e) => {
        if (cancelled) return;
        setRobotId(null);
        setPairError(e.message || "Could not find a robot for this session.");
      });
    return () => { cancelled = true; };
  }, [api, isPaired, sessionId]);

  async function handlePlaySentence() {
    const sentence = sentences[currentIdx];
    if (!sentence) return;
    setPlayError(null);

    if (playMode === "browser") {
      if (!sentence.audioUrl) {
        setPlayError("Audio isn't available for this sentence yet.");
        return;
      }
      const audio = audioPlayerRef.current ?? new Audio();
      audioPlayerRef.current = audio;
      audio.src = sentence.audioUrl.startsWith("http") ? sentence.audioUrl : `${api.baseUrl}${sentence.audioUrl}`;
      try {
        await audio.play();
      } catch {
        setPlayError("Could not play audio in this browser.");
      }
    } else {
      if (!isPaired || !robotId) {
        setPlayError("Pair a robot first.");
        return;
      }
      try {
        await api.queueRsrSentence(robotId, sentence.id);
        setSentToRobotId(sentence.id);
        setTimeout(() => setSentToRobotId((cur) => (cur === sentence.id ? null : cur)), 2000);
      } catch (e) {
        setPlayError(`Could not send sentence to robot: ${e.message}`);
      }
    }
  }

  async function startAssessment() {
    if (!ageMonths || parseInt(ageMonths, 10) < 1) {
      setError("Please enter a valid age in months.");
      return;
    }
    setError(null);

    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      streamRef.current = stream;

      const recorder = new MediaRecorder(stream, { mimeType: preferredMimeType() });
      chunksRef.current = [];
      recorder.ondataavailable = (e) => { if (e.data.size > 0) chunksRef.current.push(e.data); };
      mediaRecorderRef.current = recorder;

      recorder.start(100);
      sessionStartRef.current = Date.now();

      setCurrentIdx(0);
      setMarkers([]);
      setPhase(PHASE.ASSESSMENT);
    } catch {
      setError("Microphone access is required. Please allow microphone access and try again.");
    }
  }

  function handleStartRecording() {
    setRecordingStart(Date.now() - sessionStartRef.current);
    setIsRecording(true);
  }

  function handleStopRecording() {
    const startMs = recordingStart;
    const endMs = Date.now() - sessionStartRef.current;
    const sentenceId = currentIdx + 1;

    setMarkers((prev) => [
      ...prev.filter((m) => m.sentence_id !== sentenceId),
      { sentence_id: sentenceId, start_ms: startMs, end_ms: endMs },
    ]);
    setIsRecording(false);
    setRecordingStart(null);
  }

  async function handleSubmit() {
    if (mediaRecorderRef.current && mediaRecorderRef.current.state !== "inactive") {
      mediaRecorderRef.current.stop();
    }
    if (streamRef.current) {
      streamRef.current.getTracks().forEach((t) => t.stop());
    }

    if (markers.length === 0) {
      setError("At least one sentence must be recorded before submitting.");
      return;
    }

    setPhase(PHASE.SUBMITTING);
    setError(null);

    await new Promise((resolve) => setTimeout(resolve, 300));

    const mimeType = preferredMimeType() || "audio/webm";
    const blob = new Blob(chunksRef.current, { type: mimeType });
    const ext = mimeType.includes("ogg") ? "ogg" : "webm";

    const formData = new FormData();
    formData.append("sessionAudio", blob, `session.${ext}`);
    formData.append("age", ageMonths);
    formData.append("percentile", percentile);
    formData.append("markers", JSON.stringify(markers));

    try {
      const data = await api.analyzeRsr(formData);
      setResults(data);
      setPhase(PHASE.RESULTS);
    } catch (e) {
      setError(`Analysis failed: ${e.message}`);
      setPhase(PHASE.ASSESSMENT);
    }
  }

  function restart() {
    setPhase(PHASE.SETUP);
    setResults(null);
    setMarkers([]);
    setCurrentIdx(0);
    setError(null);
    chunksRef.current = [];
  }

  const markedCount = markers.length;
  const currentMarker = markers.find((m) => m.sentence_id === currentIdx + 1);

  return (
    <div className="max-w-3xl mx-auto px-4 py-8">
      <h1 className="text-2xl font-bold mb-1">RSR Assessment</h1>
      <p className="text-gray-500 mb-6 text-sm">Redmond Sentence Recall — automated scoring via AutoRSR</p>

      {error && (
        <div className="mb-4 rounded-lg border border-red-200 bg-red-50 px-4 py-3 text-sm text-red-700">
          {error}
        </div>
      )}

      {/* ── Setup ─────────────────────────────────────────────────────────── */}
      {phase === PHASE.SETUP && (
        <div className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
          <h2 className="text-lg font-semibold mb-4">Session Setup</h2>
          <div className="grid grid-cols-1 gap-4 sm:grid-cols-2">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Age (months) <span className="text-red-500">*</span>
              </label>
              <input
                type="number"
                min="24"
                max="120"
                value={ageMonths}
                onChange={(e) => setAgeMonths(e.target.value)}
                placeholder="e.g. 72 for 6 years"
                className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Percentile threshold
              </label>
              <select
                value={percentile}
                onChange={(e) => setPercentile(e.target.value)}
                className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
              >
                <option value="5">5th</option>
                <option value="10">10th</option>
                <option value="16">16th</option>
                <option value="25">25th</option>
              </select>
            </div>
          </div>

          <div className="mt-6">
            <h3 className="text-sm font-medium text-gray-700 mb-2">Assessment sentences ({sentences.length} total)</h3>
            <ol className="text-sm text-gray-500 space-y-1 max-h-48 overflow-y-auto rounded-lg border border-gray-100 bg-gray-50 p-3">
              {sentences.map((s, i) => (
                <li key={s.id} className="leading-snug">
                  <span className="font-mono text-gray-400 mr-2">{i + 1}.</span>{s.text}
                </li>
              ))}
            </ol>
          </div>

          <div className="mt-6">
            <h3 className="text-sm font-medium text-gray-700 mb-2">Sentence playback</h3>
            <div className="flex gap-2">
              <button
                type="button"
                onClick={() => setPlayMode("browser")}
                className={`flex-1 rounded-lg border py-2 text-sm font-medium transition-colors ${
                  playMode === "browser"
                    ? "border-blue-600 bg-blue-50 text-blue-700"
                    : "border-gray-300 text-gray-600 hover:bg-gray-50"
                }`}
              >
                Web terminal
              </button>
              <button
                type="button"
                onClick={() => setPlayMode("robot")}
                className={`flex-1 rounded-lg border py-2 text-sm font-medium transition-colors ${
                  playMode === "robot"
                    ? "border-blue-600 bg-blue-50 text-blue-700"
                    : "border-gray-300 text-gray-600 hover:bg-gray-50"
                }`}
              >
                Robot face
              </button>
            </div>

            {playMode === "robot" && (
              <div className="mt-3 flex items-center gap-2">
                <button
                  type="button"
                  onClick={() => setShowPairCard(true)}
                  className="rounded-lg bg-gray-800 px-4 py-2 text-sm font-semibold text-white hover:bg-gray-900 transition-colors"
                >
                  {isPaired ? "Manage Pairing" : "Pair Robot"}
                </button>
              </div>
            )}
            {playMode === "robot" && isPaired && robotId && (
              <p className="mt-2 text-sm text-green-600">Robot connected.</p>
            )}
            {playMode === "robot" && pairError && (
              <p className="mt-2 text-sm text-red-500">{pairError}</p>
            )}
          </div>

          <button
            onClick={startAssessment}
            className="mt-6 w-full rounded-lg bg-blue-600 py-2.5 text-sm font-semibold text-white hover:bg-blue-700 transition-colors"
          >
            Start Assessment
          </button>
        </div>
      )}

      {showPairCard && (
        <div
          className="fixed inset-0 flex items-center justify-center bg-black/40 z-50"
          role="dialog"
          aria-modal="true"
          aria-label="Pair a robot for this assessment"
        >
          <PairRobotCard
            onCancel={() => setShowPairCard(false)}
            onPaired={() => setShowPairCard(false)}
            onUnpaired={() => setShowPairCard(false)}
          />
        </div>
      )}

      {/* ── Assessment ────────────────────────────────────────────────────── */}
      {phase === PHASE.ASSESSMENT && (
        <div className="space-y-4">
          <div className="flex items-center justify-between text-sm text-gray-500">
            <span>Sentence {currentIdx + 1} of {sentences.length}</span>
            <span>{markedCount} recorded</span>
          </div>

          <div className="w-full bg-gray-200 rounded-full h-1.5">
            <div
              className="bg-blue-500 h-1.5 rounded-full transition-all"
              style={{ width: `${(currentIdx / sentences.length) * 100}%` }}
            />
          </div>

          <div className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
            <p className="text-xs font-semibold uppercase tracking-wide text-gray-400 mb-3">
              Read this sentence aloud to the student:
            </p>
            <p className="text-xl font-medium text-gray-800 leading-relaxed">
              {sentences[currentIdx]?.text}
            </p>

            <div className="mt-3 flex items-center gap-3">
              <button
                type="button"
                onClick={handlePlaySentence}
                className="flex items-center gap-2 rounded-lg border border-gray-300 px-3 py-1.5 text-sm font-medium text-gray-600 hover:bg-gray-50 transition-colors"
              >
                ▶ Play {playMode === "robot" ? "on robot" : "in browser"}
              </button>
              {sentToRobotId === sentences[currentIdx]?.id && (
                <span className="text-sm text-green-600 font-medium">Sent to robot</span>
              )}
            </div>
            {playError && <p className="mt-2 text-sm text-red-500">{playError}</p>}

            <div className="mt-6 flex items-center gap-3">
              {!isRecording ? (
                <button
                  onClick={handleStartRecording}
                  className="flex items-center gap-2 rounded-lg bg-red-500 px-4 py-2.5 text-sm font-semibold text-white hover:bg-red-600 transition-colors"
                >
                  <span className="h-2.5 w-2.5 rounded-full bg-white animate-pulse" />
                  {currentMarker ? "Re-record" : "Record Response"}
                </button>
              ) : (
                <button
                  onClick={handleStopRecording}
                  className="flex items-center gap-2 rounded-lg bg-gray-800 px-4 py-2.5 text-sm font-semibold text-white hover:bg-gray-900 transition-colors"
                >
                  <span className="h-2.5 w-2.5 rounded-sm bg-white" />
                  Stop Recording
                </button>
              )}

              {currentMarker && !isRecording && (
                <span className="text-sm text-green-600 font-medium">Recorded</span>
              )}
            </div>

            {isRecording && (
              <p className="mt-3 text-sm text-red-500 animate-pulse">Recording in progress…</p>
            )}
          </div>

          <div className="flex gap-3">
            {currentIdx < sentences.length - 1 ? (
              <>
                <button
                  onClick={() => { if (!isRecording) setCurrentIdx((i) => i + 1); }}
                  disabled={isRecording}
                  className="flex-1 rounded-lg border border-gray-300 py-2.5 text-sm font-medium text-gray-600 hover:bg-gray-50 disabled:opacity-40 transition-colors"
                >
                  Skip
                </button>
                <button
                  onClick={() => { if (!isRecording) setCurrentIdx((i) => i + 1); }}
                  disabled={isRecording || !currentMarker}
                  className="flex-1 rounded-lg bg-blue-600 py-2.5 text-sm font-semibold text-white hover:bg-blue-700 disabled:opacity-40 transition-colors"
                >
                  Next Sentence
                </button>
              </>
            ) : (
              <button
                onClick={handleSubmit}
                disabled={isRecording || markedCount === 0}
                className="flex-1 rounded-lg bg-green-600 py-2.5 text-sm font-semibold text-white hover:bg-green-700 disabled:opacity-40 transition-colors"
              >
                Submit for Analysis
              </button>
            )}
          </div>

          {currentIdx === sentences.length - 1 && markedCount > 0 && markedCount < sentences.length && (
            <p className="text-sm text-gray-500 text-center">
              {markedCount} of {sentences.length} sentences recorded — you can still submit.
            </p>
          )}
        </div>
      )}

      {/* ── Submitting ────────────────────────────────────────────────────── */}
      {phase === PHASE.SUBMITTING && (
        <div className="rounded-xl border border-gray-200 bg-white p-12 text-center shadow-sm">
          <div className="mx-auto mb-4 h-10 w-10 animate-spin rounded-full border-4 border-blue-200 border-t-blue-600" />
          <p className="text-gray-700 font-medium">Analyzing responses…</p>
          <p className="text-sm text-gray-400 mt-1">This may take a minute while audio is transcribed and scored.</p>
        </div>
      )}

      {/* ── Results ───────────────────────────────────────────────────────── */}
      {phase === PHASE.RESULTS && results && (
        <div className="space-y-6">
          {/* PID banner */}
          <div className="rounded-xl border border-blue-200 bg-blue-50 px-6 py-4">
            <p className="text-xs font-semibold uppercase tracking-wide text-blue-500 mb-1">Participant ID</p>
            <p className="text-3xl font-mono font-bold text-blue-800 tracking-widest">{results.pid}</p>
            <p className="text-xs text-blue-500 mt-1">Save this ID to look up this assessment later.</p>
          </div>

          {/* Score + decision */}
          <div className="rounded-xl border bg-white p-6 shadow-sm">
            <div className="flex items-center justify-between flex-wrap gap-4">
              <div>
                <p className="text-sm text-gray-500">Total Score</p>
                <p className="text-4xl font-bold text-gray-900">{results.totalScore}</p>
              </div>
              <div className={`rounded-full px-6 py-2 text-lg font-bold ${
                results.decision === "Pass"
                  ? "bg-green-100 text-green-700"
                  : results.decision === "Fail"
                  ? "bg-red-100 text-red-700"
                  : "bg-gray-100 text-gray-600"
              }`}>
                {results.decision}
              </div>
            </div>
            <div className="mt-4 text-sm text-gray-500">
              Age: {ageMonths} months &nbsp;·&nbsp; Percentile threshold: {percentile}th
            </div>
          </div>

          {/* Sentence breakdown */}
          <div className="rounded-xl border bg-white shadow-sm overflow-hidden">
            <div className="px-6 py-4 border-b">
              <h2 className="font-semibold text-gray-800">Sentence Breakdown</h2>
            </div>
            <div className="divide-y">
              {results.sentences.map((s) => (
                <div key={s.id} className="px-6 py-4">
                  <div className="flex items-center justify-between mb-1">
                    <span className="text-xs font-mono text-gray-400">#{s.id}</span>
                    <div className="flex items-center gap-3">
                      <span className="text-xs text-gray-500">{s.errors} error{s.errors !== 1 ? "s" : ""}</span>
                      <span className={`text-sm font-bold ${
                        s.score === 2 ? "text-green-600" : s.score === 1 ? "text-yellow-600" : "text-red-500"
                      }`}>
                        {s.score}/2
                      </span>
                    </div>
                  </div>
                  <p className="text-sm text-gray-700">{s.groundTruth}</p>
                  {s.response
                    ? <p className="text-sm text-gray-400 mt-0.5 italic">"{s.response}"</p>
                    : <p className="text-sm text-gray-300 mt-0.5 italic">No response recorded</p>
                  }
                </div>
              ))}
            </div>
          </div>

          <button
            onClick={restart}
            className="w-full rounded-lg border border-gray-300 py-2.5 text-sm font-medium text-gray-600 hover:bg-gray-50 transition-colors"
          >
            New Assessment
          </button>
        </div>
      )}
    </div>
  );
}
