// Bytes <-> base64, for the remote shell: SignalR's JSON protocol carries byte[] as base64 strings.

export function bytesToBase64(bytes) {
  // Chunked so a large array can't overflow the argument limit of String.fromCharCode.
  let binary = "";
  for (let i = 0; i < bytes.length; i += 0x8000) {
    binary += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  }
  return btoa(binary);
}

export function base64ToBytes(text) {
  const binary = atob(text);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

// Splits terminal input into base64 pieces of at most `maxBytes` raw bytes each. The backend rejects
// (and drops the connection on) messages over its receive limit, so a big paste must not go as one.
export function encodeInput(text, maxBytes = 8192) {
  const bytes = new TextEncoder().encode(text);
  const pieces = [];
  for (let i = 0; i < bytes.length; i += maxBytes) {
    pieces.push(bytesToBase64(bytes.subarray(i, i + maxBytes)));
  }
  return pieces;
}
