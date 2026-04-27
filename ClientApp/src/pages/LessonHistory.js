import { useParams } from "react-router-dom";
import { useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

export default function LessonHistory() {
    const { sessionId } = useParams();
    const api = useApiClient();

    const [conversation, setConversation] = useState([]);
    const [isLoadingHistory, setIsLoadingHistory] = useState(false);

    useEffect(() => {
        async function loadHistory() {
            const fakeData = [
                { id: 1, type: "robot", text: "Say the word 'right'", ts: "10:01 AM" },
                { id: 2, type: "student", text: "wight", ts: "10:02 AM" },
                { id: 3, type: "robot", text: "Try again", ts: "10:02 AM" },
                { id: 4, type: "student", text: "right", ts: "10:03 AM" },
                { id: 5, type: "note", text: "Student corrected pronunciation after second attempt", ts: "10:04 AM" },
            ];

            const isGuid =
                /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i.test(sessionId);

            if (!isGuid) {
                setConversation(fakeData);
                return;
            }

            try {
                setIsLoadingHistory(true);

                const data = await api.getLessonInteractions(sessionId);
                const interactionArray = Array.isArray(data) ? data : [];

                const mapped = interactionArray
                    .map((item, index) => {
                        const interactionType = String(item.interactionType ?? "").toLowerCase();

                        return {
                            id: item.id ?? `interaction-${index}`,
                            type:
                                interactionType === "speaker"
                                    ? "student"
                                    : interactionType === "note"
                                    ? "note"
                                    : "robot",
                            text: item.studentResponse || item.dialogTurn || "",
                            ts: item.timestamp
                                ? new Date(item.timestamp).toLocaleTimeString([], {
                                      hour: "numeric",
                                      minute: "2-digit",
                                  })
                                : "",
                        };
                    })
                    .filter((item) => item.text);

                setConversation(mapped.length > 0 ? mapped : fakeData);
            } catch (err) {
                console.error("Failed to load lesson history:", err);
                setConversation(fakeData);
            } finally {
                setIsLoadingHistory(false);
            }
        }

        loadHistory();
    }, [api, sessionId]);

    function renderMessage(item) {
        if (item.type === "robot") {
            return (
                <div key={item.id} className="flex flex-col items-start max-w-[75%]">
                    <p className="mb-1 text-xs font-semibold text-gray-600">Robot</p>
                    <div className="rounded-2xl rounded-bl-md bg-gray-200 px-4 py-3 text-sm text-gray-900 shadow-sm">
                        <p>{item.text}</p>
                    </div>
                    <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
                </div>
            );
        }

        if (item.type === "student") {
            return (
                <div key={item.id} className="ml-auto flex max-w-[75%] flex-col items-end">
                    <p className="mb-1 text-xs font-semibold text-gray-600">Student</p>
                    <div className="rounded-2xl rounded-br-md bg-green-500 px-4 py-3 text-sm text-white shadow-sm">
                        <p>{item.text}</p>
                    </div>
                    <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
                </div>
            );
        }

        return (
            <div key={item.id} className="mx-auto flex max-w-[70%] flex-col items-center">
                <p className="mb-1 text-xs font-semibold text-gray-600">SLP Note</p>
                <div className="rounded-xl border border-yellow-300 bg-yellow-100 px-4 py-3 text-center text-sm text-gray-900 shadow-sm">
                    <p>{item.text}</p>
                </div>
                <p className="mt-1 text-[11px] text-gray-500">{item.ts}</p>
            </div>
        );
    }

    return (
        <div className="mt-4 rounded-lg border p-6 shadow-sm">
            <div className="rounded-lg border p-4 shadow-sm">
                <div className="flex items-center justify-between">
                    <div>
                        <h1 className="text-lg font-semibold text-gray-900">
                            Lesson Transcript
                        </h1>
                        <p className="mt-1 text-xs text-gray-500">
                            Session: {sessionId}
                        </p>
                    </div>

                    {isLoadingHistory && (
                        <p className="text-xs font-semibold text-gray-500">
                            Loading...
                        </p>
                    )}
                </div>

                <div className="mt-4 h-[750px] space-y-4 overflow-y-auto rounded-2xl border bg-white p-4">
                    {conversation.length === 0 ? (
                        <p className="text-sm text-gray-500">No activity found.</p>
                    ) : (
                        conversation.map((item) => renderMessage(item))
                    )}
                </div>
            </div>
        </div>
    );
}