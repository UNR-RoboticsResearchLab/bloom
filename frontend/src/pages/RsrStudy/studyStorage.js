// IndexedDB stores each audio chunk as it arrives, including speech before item 1.
// Kept outside localStorage: audio can exceed its quota within minutes.
const DB = 'bloom-rsr-study';
function database() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB, 1);
    request.onupgradeneeded = () => {
      request.result.createObjectStore('sessions', { keyPath: 'id' });
      request.result.createObjectStore('chunks', { keyPath: ['sessionId', 'index'] });
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}
async function write(store, value) {
  const db = await database();
  try {
    await new Promise((resolve, reject) => {
      const tx = db.transaction(store, 'readwrite');
      tx.objectStore(store).put(value);
      tx.oncomplete = resolve;
      tx.onerror = () => reject(tx.error);
      tx.onabort = () => reject(tx.error || new Error('Local save aborted.'));
    });
  } finally { db.close(); }
}
export const saveSession = session => write('sessions', session);
export const saveChunk = (sessionId, index, blob) => write('chunks', { sessionId, index, blob });
export async function readSessions() {
  const db = await database();
  try {
    return await new Promise((resolve, reject) => {
      const request = db.transaction('sessions').objectStore('sessions').getAll();
      request.onsuccess = () => resolve(request.result.sort((a,b) => b.createdAt.localeCompare(a.createdAt)));
      request.onerror = () => reject(request.error);
    });
  } finally { db.close(); }
}
export async function readAudio(id, mimeType) {
  const db = await database();
  try {
    const chunks = await new Promise((resolve, reject) => {
      const request = db.transaction('chunks').objectStore('chunks')
        .getAll(IDBKeyRange.bound([id, 0], [id, Number.MAX_SAFE_INTEGER]));
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => reject(request.error);
    });
    return new Blob(chunks.map(c => c.blob), { type: mimeType });
  } finally { db.close(); }
}
export function download(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = filename; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export async function replaceAudio(id, blob) {
  const db = await database();
  try {
    await new Promise((resolve, reject) => {
      const tx = db.transaction('chunks', 'readwrite');
      const store = tx.objectStore('chunks');
      store.delete(IDBKeyRange.bound([id, 0], [id, Number.MAX_SAFE_INTEGER]));
      store.put({ sessionId: id, index: 0, blob });
      tx.oncomplete = resolve;
      tx.onerror = () => reject(tx.error);
      tx.onabort = () => reject(tx.error || new Error('Audio recovery save aborted.'));
    });
  } finally { db.close(); }
}
