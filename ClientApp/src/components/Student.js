import { useParams } from "react-router-dom";

function getInitials(name) {
    return name
        .split(" ")
        .map(word => word[0])
        .join("")
        .toUpperCase();
}


export default function Student() {
    const { name } = useParams();
    
    const initials = getInitials(name);
    return (
        <div className="rounded-lg border p-6 shadow-sm">
            <p className="text-sm font-semibold text-gray-900">
                Students &gt; {name}
            </p>

            <div className="mt-3 rounded-lg border p-4 shadow-sm">
                <div className="flex items-center gap-4">
                    <div className="inline-flex h-10 w-10 items-center justify-center rounded-full border border-gray-400 bg-white">
                        <span className="text-sm font-semibold text-gray-900">
                            {initials}
                        </span>
                    </div>

                    <div className="leading-tight">
                        <p className="text-sm font-semibold text-gray-900">
                            {name}
                        </p>
                        <p className="text-xs text-gray-600">
                            level 2
                        </p>
                        <p className="text-xs text-gray-600">
                            Active
                        </p>
                    </div>
                </div>
            </div>
        </div>
    );
}

