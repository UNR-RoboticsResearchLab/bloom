import Card from "../components/ui/Card";

export function StudentCard({ name, email, active, completed, selected, onClick }) {
  return (
    <Card as="button" interactive selected={selected} onClick={onClick}>
      <div className="flex items-start justify-between gap-6">
        <div>
          <div className="text-base font-semibold text-gray-900">{name}</div>
          <div className="mt-2 flex gap-6 text-xs text-gray-600">
            <div>
              {/* <span className="font-medium text-gray-700">Active:</span>{" "}
              {active?.join(", ") || "None"} */}
            </div>
            <div>
              {/* <span className="font-medium text-gray-700">Completed:</span>{" "}
              {completed?.join(", ") || "None"} */}
            </div>
          </div>
        </div>

        {/* <div className="text-sm text-gray-500">{email}</div> */}
      </div>
    </Card>
  );
}
