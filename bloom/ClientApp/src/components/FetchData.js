import { useEffect, useState } from "react";

export default function FetchData() {
  const [forecasts, setForecasts] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    let active = true;
    (async () => {
      try {
        const res = await fetch("weatherforecast");
        const data = await res.json();
        if (active) {
          setForecasts(data);
          setLoading(false);
        }
      } catch (e) {
        if (active) setLoading(false);
        console.error("Failed to load weatherforecast", e);
      }
    })();
    return () => {
      active = false;
    };
  }, []);

  const renderTable = (rows) => (
    <table className="table table-striped" aria-labelledby="tableLabel">
      <thead>
        <tr>
          <th>Date</th>
          <th>Temp. (C)</th>
          <th>Temp. (F)</th>
          <th>Summary</th>
        </tr>
      </thead>
      <tbody>
        {rows.map((f, i) => (
          <tr key={f.date ?? i}>
            <td>{f.date}</td>
            <td>{f.temperatureC}</td>
            <td>{f.temperatureF}</td>
            <td>{f.summary}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );

  return (
    <div>
      <h1 id="tableLabel">Weather forecast</h1>
      <p>This component demonstrates fetching data from the server.</p>
      {loading ? <p><em>Loading...</em></p> : renderTable(forecasts)}
    </div>
  );
}
