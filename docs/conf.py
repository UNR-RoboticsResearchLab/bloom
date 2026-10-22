"""Sphinx configuration; the repository root is the documentation source root."""
from pathlib import Path

project = "Bloom"
author = "UNR Robotics Research Lab"
extensions = ["myst_parser"]
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
root_doc = "docs/index"
exclude_patterns = [
    ".git/**", ".test-tools/**", ".pnpm-store/**", ".agents/**", ".codex/**",
    "**/node_modules/**", "**/build/**", "**/bin/**", "**/obj/**",
    "**/install/**", "**/log/**", "**/__pycache__/**", "**/AGENTS.md",
    "docs/_build/**", "docs/.venv/**", "backend/App_Data/**",
]
html_theme = "shibuya"
html_title = "Bloom documentation"
html_static_path = []
myst_heading_anchors = 3


def setup(app):
    # Include every repository Markdown guide in the documentation tree, including
    # newly added guides. Agent instructions and generated/dependency files are
    # excluded above and are not product documentation.
    app.connect("builder-inited", generate_catalog)
    app.connect("source-read", provide_missing_titles)


def generate_catalog(app):
    from sphinx.util.matching import Matcher
    excluded = Matcher(exclude_patterns)
    root = Path(app.srcdir)
    import os
    pages = []
    for directory, children, filenames in os.walk(root):
        parent = Path(directory)
        children[:] = [name for name in children
                       if not excluded((parent / name).relative_to(root).as_posix() + "/")]
        for filename in filenames:
            path = parent / filename
            relative = path.relative_to(root).as_posix()
            if path.suffix == ".md" and not excluded(relative) and relative != "docs/rsr-study.md":
                pages.append(path.relative_to(root).with_suffix("").as_posix())
    pages.sort()
    catalog = root / "docs" / "_catalog.rst"
    catalog.write_text(
        "Repository guides\n=================\n\n"
        ".. toctree::\n   :maxdepth: 1\n\n"
        + "".join(f"   /{page}\n" for page in pages),
        encoding="utf-8",
    )


def provide_missing_titles(app, docname, source):
    """Make older title-less guides and empty placeholders navigable."""
    path = Path(app.env.doc2path(docname))
    if path.suffix != ".md":
        return
    text = source[0]
    if not text.lstrip().startswith("# "):
        title = docname.replace("/README", "").replace("/", " / ")
        source[0] = f"# {title}\n\n" + (text or "This documentation source is currently empty.\n")
