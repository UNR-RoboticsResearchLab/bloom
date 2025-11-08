// Container to keep a consistent look for sections
export default function SectionCard({ title, actions, children }) {
  return (
    <section className="rounded-xl bg-white p-4 shadow">
      <header className="mb-3 flex items-center justify-between">
        <h3 className="text-base font-semibold text-gray-900">{title}</h3>
        {actions}
      </header>
      {children}
    </section>
  );
}
