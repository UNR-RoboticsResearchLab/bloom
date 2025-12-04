import { useState } from "react";

export default function Counter() {
  const [count, setCount] = useState(0);

  return (
    <div>
      <h1>Counter</h1>
      <p>This is a simple example of a React component.</p>
      <p aria-live="polite">
        Current count: <strong>{count}</strong>
      </p>
      <button className="btn btn-primary" onClick={() => setCount(c => c + 1)}>
        Increment
      </button>
    </div>
  );
}
