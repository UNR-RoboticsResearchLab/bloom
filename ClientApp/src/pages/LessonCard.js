export function LessonCard({ title, description, onClick }) {
    return (
        <div onClick={onClick} className="cursor-pointer rounded-lg border border-gray-200 p-4 shadow-sm hover:bg-blue-200">
            <div className="flex items-center justify-between">
                <p className="text-lg font-medium text-gray-900">{title}</p> 
                <p className="text-sm text-gray-600">{description}</p>
            </div>
        </div>
    );
}