// All event times use the recorder's monotonic clock, never wall-clock differences.
export const CONDITIONS = [
  { id: 'condition-1', label: 'Condition 1', delivery: 'examiner' },
  { id: 'condition-2', label: 'Condition 2', delivery: 'browser' },
  { id: 'condition-3', label: 'Condition 3', delivery: 'robot' },
];

export function responseMarkers(attempts) {
  // Only explicitly marked response windows go to ASR; the full audio is retained.
  const latest = new Map();
  attempts.filter(a => !a.excluded && a.responseStart != null && a.end > a.responseStart)
    .forEach(a => latest.set(a.sentenceId, {
      sentence_id: a.sentenceId, start_ms: a.responseStart, end_ms: a.end,
    }));
  return [...latest.values()];
}

export function reviewedScores(sentences, results, overrides) {
  return sentences.map(s => {
    const wasAnalyzed = !results?.analyzedSentenceIds || results.analyzedSentenceIds.includes(s.id);
    const original = wasAnalyzed ? results?.sentences?.find(r => String(r.id) === String(s.id)) : null;
    return { id: s.id, automatic: original?.score ?? null,
      reviewed: overrides[s.id]?.score ?? original?.score ?? null,
      reason: overrides[s.id]?.reason ?? '' };
  });
}
