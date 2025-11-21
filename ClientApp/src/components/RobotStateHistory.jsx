import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";

/**
 * RobotStateHistory Component
 * Displays historical robot state snapshots for a given session
 *
 * Data is fetched every 10th state change per robot, showing status, tasks, and timestamps
 * Used for analyzing robot behavior and session performance
 *
 * @param {string} sessionId - Optional session ID passed as prop. If not provided, will attempt to get from URL params
 */
export default function RobotStateHistory({ sessionId: propSessionId }) {
  const apiClient = useApiClient();
  const { sessionId: paramSessionId } = useParams();
  const sessionId = propSessionId || paramSessionId;
  const [history, setHistory] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [sortBy, setSortBy] = useState("timestamp"); // "timestamp", "robotId", "status"
  const [sortOrder, setSortOrder] = useState("desc"); // "asc", "desc"

  useEffect(() => {
    if (!sessionId) {
      setError("Session ID is required");
      setLoading(false);
      return;
    }

    let active = true;

    (async () => {
      try {
        setLoading(true);
        const data = await apiClient.getSessionHistory(sessionId);

        if (active) {
          // Extract history array from response
          const historyData = data.history || [];
          setHistory(historyData);
          setError(null);
          setLoading(false);
        }
      } catch (err) {
        if (active) {
          setError(err.message || "Failed to load state history");
          setHistory([]);
          setLoading(false);
        }
      }
    })();

    return () => {
      active = false;
    };
  }, [sessionId, apiClient]);

  /**
   * Sort history data based on current sort settings
   */
  const sortedHistory = [...history].sort((a, b) => {
    let compareValue = 0;

    switch (sortBy) {
      case "timestamp":
        compareValue = new Date(a.timestamp) - new Date(b.timestamp);
        break;
      case "robotId":
        compareValue = a.robotId.localeCompare(b.robotId);
        break;
      case "status":
        compareValue = a.status.localeCompare(b.status);
        break;
      default:
        compareValue = 0;
    }

    return sortOrder === "asc" ? compareValue : -compareValue;
  });

  /**
   * Get status badge styling based on robot status
   */
  const getStatusBadge = (status) => {
    const statusLower = status?.toLowerCase() || "unknown";
    let bgColor = "bg-gray-100";
    let textColor = "text-gray-700";

    switch (statusLower) {
      case "waiting":
        bgColor = "bg-blue-50";
        textColor = "text-blue-700";
        break;
      case "speaking":
        bgColor = "bg-green-50";
        textColor = "text-green-700";
        break;
      case "loading":
        bgColor = "bg-yellow-50";
        textColor = "text-yellow-700";
        break;
      case "error":
        bgColor = "bg-red-50";
        textColor = "text-red-700";
        break;
      default:
        break;
    }

    return (
      <span className={`rounded-full ${bgColor} ${textColor} px-2 py-0.5 text-xs font-semibold`}>
        {status}
      </span>
    );
  };

  /**
   * Format ISO datetime to readable local string
   */
  const formatDate = (isoString) => {
    if (!isoString) return "N/A";
    return new Date(isoString).toLocaleString();
  };

  /**
   * Handle sort column click
   */
  const handleSort = (column) => {
    if (sortBy === column) {
      // Toggle sort order if same column
      setSortOrder(sortOrder === "asc" ? "desc" : "asc");
    } else {
      // Set new sort column
      setSortBy(column);
      setSortOrder("asc");
    }
  };

  /**
   * Render sort indicator arrow
   */
  const renderSortIndicator = (column) => {
    if (sortBy !== column) return null;
    return sortOrder === "asc" ? " ↑" : " ↓";
  };

  /**
   * Render history table
   */
  const renderHistoryTable = () => (
    <div className="overflow-x-auto rounded-lg border border-gray-200">
      <table className="w-full">
        <thead className="bg-gray-50 border-b border-gray-200">
          <tr>
            <th className="px-6 py-3 text-left text-xs font-semibold text-gray-700">
              Session ID
            </th>
            <th
              onClick={() => handleSort("robotId")}
              className="px-6 py-3 text-left text-xs font-semibold text-gray-700 cursor-pointer hover:bg-gray-100 transition"
            >
              Robot ID{renderSortIndicator("robotId")}
            </th>
            <th
              onClick={() => handleSort("status")}
              className="px-6 py-3 text-left text-xs font-semibold text-gray-700 cursor-pointer hover:bg-gray-100 transition"
            >
              Status{renderSortIndicator("status")}
            </th>
            <th className="px-6 py-3 text-left text-xs font-semibold text-gray-700">
              Current Task
            </th>
            <th
              onClick={() => handleSort("timestamp")}
              className="px-6 py-3 text-left text-xs font-semibold text-gray-700 cursor-pointer hover:bg-gray-100 transition"
            >
              Timestamp{renderSortIndicator("timestamp")}
            </th>
          </tr>
        </thead>
        <tbody className="divide-y divide-gray-200">
          {sortedHistory.length > 0 ? (
            sortedHistory.map((entry) => (
              <tr key={entry.id} className="hover:bg-gray-50 transition">
                <td className="px-6 py-3 text-sm text-gray-900 font-mono">
                  {entry.robotSessionId}
                </td>
                <td className="px-6 py-3 text-sm text-gray-900 font-mono">
                  {entry.robotId}
                </td>
                <td className="px-6 py-3 text-sm">
                  {getStatusBadge(entry.status)}
                </td>
                <td className="px-6 py-3 text-sm text-gray-700">
                  {entry.currentTask || "N/A"}
                </td>
                <td className="px-6 py-3 text-sm text-gray-600">
                  {formatDate(entry.timestamp)}
                </td>
              </tr>
            ))
          ) : (
            <tr>
              <td colSpan="5" className="px-6 py-8 text-center text-sm text-gray-500">
                No state history available for this session
              </td>
            </tr>
          )}
        </tbody>
      </table>
    </div>
  );

  /**
   * Render empty state
   */
  const renderEmpty = () => (
    <div className="rounded-lg bg-blue-50 p-6 border border-blue-200">
      <h3 className="text-base font-semibold text-blue-900">No History Available</h3>
      <p className="mt-2 text-sm text-blue-700">
        State history is recorded every 10th state change per robot during an active session.
      </p>
    </div>
  );

  /**
   * Render error state
   */
  const renderError = () => (
    <div className="rounded-lg bg-red-50 p-6 border border-red-200">
      <h3 className="text-base font-semibold text-red-900">Error Loading History</h3>
      <p className="mt-2 text-sm text-red-700">{error}</p>
    </div>
  );

  /**
   * Render loading state
   */
  const renderLoading = () => (
    <div className="flex items-center justify-center py-12">
      <div className="text-center">
        <div className="inline-block h-8 w-8 animate-spin rounded-full border-4 border-gray-300 border-t-indigo-600"></div>
        <p className="mt-4 text-sm text-gray-600">Loading state history...</p>
      </div>
    </div>
  );

  /**
   * Render stats summary
   */
  const renderStats = () => {
    if (history.length === 0) return null;

    const uniqueRobots = new Set(history.map((h) => h.robotId)).size;
    const statusCount = {};
    history.forEach((h) => {
      statusCount[h.status] = (statusCount[h.status] || 0) + 1;
    });

    return (
      <div className="mb-6 grid grid-cols-1 gap-4 sm:grid-cols-3">
        <div className="rounded-lg bg-white p-4 shadow border-l-4 border-indigo-600">
          <div className="text-sm text-gray-600">Total Snapshots</div>
          <div className="mt-1 text-2xl font-semibold text-gray-900">{history.length}</div>
        </div>
        <div className="rounded-lg bg-white p-4 shadow border-l-4 border-green-600">
          <div className="text-sm text-gray-600">Unique Robots</div>
          <div className="mt-1 text-2xl font-semibold text-gray-900">{uniqueRobots}</div>
        </div>
        <div className="rounded-lg bg-white p-4 shadow border-l-4 border-purple-600">
          <div className="text-sm text-gray-600">Status Types</div>
          <div className="mt-1 text-2xl font-semibold text-gray-900">{Object.keys(statusCount).length}</div>
          <div className="mt-2 text-xs text-gray-500">
            {Object.entries(statusCount)
              .map(([status, count]) => status + ": " + count)
              .join(", ")}
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className="min-h-screen bg-gray-50 py-8 px-4 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-6xl">
        <div className="mb-8">
          <h1 className="text-3xl font-bold tracking-tight text-gray-900">Robot State History</h1>
          <p className="mt-2 text-base text-gray-600">
            Session <code className="text-sm text-gray-500 bg-gray-100 px-2 py-1 rounded">{sessionId}</code>
          </p>
          <p className="mt-2 text-sm text-gray-600">
            Historical snapshots of robot states recorded every 10 state changes during the session.
            Click column headers to sort.
          </p>
        </div>

        {renderStats()}

        <div className="rounded-lg bg-white shadow">
          <div className="p-6">
            {error ? (
              renderError()
            ) : loading ? (
              renderLoading()
            ) : history.length > 0 ? (
              renderHistoryTable()
            ) : (
              renderEmpty()
            )}
          </div>
        </div>

        {!loading && !error && (
          <div className="mt-6 text-sm text-gray-500">
            <p>
              Showing {sortedHistory.length} historical snapshot{sortedHistory.length !== 1 ? "s" : ""}
            </p>
          </div>
        )}
      </div>
    </div>
  );
}
