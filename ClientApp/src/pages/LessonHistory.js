import { useParams } from "react-router-dom";
import { useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

export default function LessonHistory() {
    const { sessionId } = useParams();
    const api = useApiClient();

    const [conversation, setConversation] = useState([]);
    const [isLoadingHistory, setIsLoadingHistory] = useState(true);

    function formatTime(value) {
        if (!value) return "";

        const date = new Date(value);

        if (Number.isNaN(date.getTime())) return "";

        return date.toLocaleTimeString([], {
            hour: "numeric",
            minute: "2-digit",
        });
    }

    useEffect(() => {
        async function loadHistory() {
            if (!sessionId) {
                setConversation([]);
                setIsLoadingHistory(false);
                return;
            }

            try {
                setIsLoadingHistory(true);

                const data = await api.getLessonInteractions(sessionId);
                console.log("Lesson history interactions:", data);

                const interactionArray = Array.isArray(data) ? data : [];

                const mapped = interactionArray
                    .map((item, index) => {
                        const interactionType = String(
                            item.interactionType ?? item.InteractionType ?? ""
                        ).toLowerCase();

                        return {
                            id: item.id ?? item.Id ?? `interaction-${index}`,
                            type:
                                interactionType === "response" ||
                                interactionType === "student" ||
                                interactionType === "speaker"
                                    ? "student"
                                    : interactionType === "note" ||
                                    interactionType === "slpfeedback"
                                    ? "note"
                                    : "robot",
                            text:
                                item.studentResponse ??
                                item.StudentResponse ??
                                item.dialogTurn ??
                                item.DialogTurn ??
                                item.message ??
                                item.Message ??
                                "",
                            ts: formatTime(item.timestamp ?? item.Timestamp),
                        };
                    })
                    .filter((item) => item.text);

                setConversation(mapped);
            } catch (err) {
                console.error("Failed to load lesson history:", err);
                setConversation([]);
            } finally {
                setIsLoadingHistory(false);
            }
        }

        loadHistory();
    }, [api, sessionId]);

    function renderMessage(item) {
        if (item.type === "robot") {
            return (
                <div key={item.id} className="flex max-w-[75%] flex-col items-start">
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
                    {isLoadingHistory ? (
                        <p className="text-sm text-gray-500">Loading lesson history...</p>
                    ) : conversation.length === 0 ? (
                        <p className="text-sm text-gray-500">No activity found.</p>
                    ) : (
                        conversation.map((item) => renderMessage(item))
                    )}
                </div>
            </div>
        </div>
    );
}