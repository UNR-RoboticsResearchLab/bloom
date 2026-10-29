// Loaded by react-scripts before every test file.
import "@testing-library/jest-dom";
import { TextDecoder, TextEncoder } from "util";

// Browsers have these; jest's jsdom environment doesn't expose Node's.
Object.assign(global, { TextEncoder, TextDecoder });
