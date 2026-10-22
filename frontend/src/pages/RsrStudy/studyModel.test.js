import { responseMarkers, reviewedScores } from './studyModel';

it('uses response windows, preserving early audio outside ASR markers', () => {
  expect(responseMarkers([{ sentenceId: 1, start: 100, responseStart: 4000, end: 8000 }]))
    .toEqual([{ sentence_id: 1, start_ms: 4000, end_ms: 8000 }]);
});
it('uses the latest eligible attempt and ignores exclusions and incomplete windows', () => {
  expect(responseMarkers([
    { sentenceId: 1, responseStart: 0, end: 1000 },
    { sentenceId: 1, responseStart: 2000, end: 3000, excluded: true },
    { sentenceId: 2, responseStart: null, end: 4000 },
    { sentenceId: 3, responseStart: 3000, end: null },
    { sentenceId: 4, responseStart: 5000, end: 5000 },
  ])).toEqual([{ sentence_id: 1, start_ms: 0, end_ms: 1000 }]);
});
it('retains original scores and treats an override of zero as a score', () => {
  expect(reviewedScores([{id: 1}, {id: 2}], { sentences: [{id: '1', score: 2}] },
    { 1: { score: 0, reason: 'Examiner review' } })).toEqual([
      {id: 1, automatic: 2, reviewed: 0, reason: 'Examiner review'},
      {id: 2, automatic: null, reviewed: null, reason: ''},
    ]);
});

it('does not show service-generated zeroes for items that were never submitted', () => {
  expect(reviewedScores([{id:1}, {id:2}], {
    analyzedSentenceIds: [1], sentences: [{id:'1', score:2}, {id:'2', score:0}],
  }, {})[1].reviewed).toBeNull();
});
