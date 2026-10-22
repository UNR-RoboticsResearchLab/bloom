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
templates_path = ["_templates"]
html_additional_pages = {"index": "landing.html"}
html_theme = "shibuya"
html_title = "Bloom documentation"
html_static_path = []
myst_heading_anchors = 3
html_theme_options = {"globaltoc_expand_depth": 1, "toctree_maxdepth": 3}


def setup(app):
    # Keep references and release notes discoverable without flattening the
    # curated study, development, and robot guides into one repository file list.
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
            if path.suffix == ".md" and not excluded(relative):
                pages.append(path.relative_to(root).with_suffix("").as_posix())
    pages.sort()
    curated = {
        "README", "docs/rsr-study", "backend/BACKEND", "frontend/README",
        "tests/README", "robot/README", "robot/src/bloom_node/README",
        "robot/src/bloom_node/lesson", "robot/kiosk-face/README",
        "robot/llm_module/README", "robot/tts_module/README",
    }
    libraries = {
        "runtime-react": "Face runtime and loading",
        "render": "Rendering",
        "orchestrator-react": "Behavior orchestration",
        "node-graph-authoring": "Node graph authoring",
        "utils": "Shared utilities",
    }
    references, releases, additional = [], [], []
    for page in pages:
        if page in curated:
            continue
        path = Path(page)
        package = path.parent.name
        if path.name == "CHANGELOG":
            releases.append((f"{package} release notes", page))
        elif page.startswith("robot/kiosk-face/packages/"):
            references.append((libraries.get(package, package), page))
        else:
            label = "Robot interface (placeholder)" if page == "robot/src/bloom_interface/README" else page
            additional.append((label, page))

    def tree(title, entries):
        if not entries:
            return ""
        return (f"{title}\n{'-' * len(title)}\n\n.. toctree::\n   :maxdepth: 1\n\n"
                + "".join(f"   {label} </{page}>\n" for label, page in entries) + "\n")

    (root / "docs" / "_catalog.rst").write_text(
        'Face libraries and additional guides\n====================================\n\n'
        + tree("Face libraries", references) + tree("Additional guides", additional),
        encoding="utf-8",
    )
    (root / "docs" / "_releases.rst").write_text(
        "Package release notes\n=====================\n\n"
        "Historical changes to the libraries used by the robot face.\n\n"
        + ".. toctree::\n   :maxdepth: 1\n\n"
        + "".join(f"   {label} </{page}>\n" for label, page in releases),
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
