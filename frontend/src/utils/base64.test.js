import { base64ToBytes, bytesToBase64, encodeInput } from "./base64";

describe("base64", () => {
  it("round-trips every byte value (terminal output is arbitrary bytes)", () => {
    const all = Uint8Array.from({ length: 256 }, (_, i) => i);
    expect(base64ToBytes(bytesToBase64(all))).toEqual(all);
    expect(bytesToBase64(new Uint8Array([]))).toBe("");
    expect(bytesToBase64(new TextEncoder().encode("foobar"))).toBe("Zm9vYmFy");
  });

  it("handles arrays larger than String.fromCharCode's argument limit", () => {
    const big = new Uint8Array(300000).fill(65);
    expect(base64ToBytes(bytesToBase64(big)).length).toBe(300000);
  });

  it("encodes input as UTF-8, not UTF-16 or Latin-1", () => {
    const [piece] = encodeInput("é─");
    expect(Array.from(base64ToBytes(piece))).toEqual([0xc3, 0xa9, 0xe2, 0x94, 0x80]);
  });

  it("splits a big paste into pieces under the limit and keeps every byte in order", () => {
    const text = "x".repeat(20000);
    const pieces = encodeInput(text, 8192);
    expect(pieces).toHaveLength(3);
    const sizes = pieces.map((p) => base64ToBytes(p).length);
    expect(sizes).toEqual([8192, 8192, 3616]);
    expect(new TextDecoder().decode(Uint8Array.from(pieces.flatMap((p) => [...base64ToBytes(p)])))).toBe(text);
    expect(encodeInput("")).toEqual([]);
  });
});
