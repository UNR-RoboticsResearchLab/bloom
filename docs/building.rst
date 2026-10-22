Build the documentation
=======================

Run these commands from the Bloom repository root with Python 3.11 or newer::

   python3 -m venv docs/.venv
   docs/.venv/bin/python -m pip install -r docs/requirements.txt
   docs/.venv/bin/python -m sphinx -b html -c docs . docs/_build/html

Open ``docs/_build/html/docs/index.html`` to browse the generated documentation.
The RSR study guide is at ``docs/_build/html/docs/rsr-study.html``.
The HTML output, Python environment, and generated navigation catalog are ignored
by Git. Rebuild after editing documentation sources.

Sphinx's MyST parser turns Markdown guides into HTML pages. Repository Markdown
files are discovered automatically and listed under Repository guides. Sphinx
reStructuredText pages live in ``docs/``. Agent instructions, dependency trees,
private study archives, and generated files are excluded.

The documentation workflow builds the site on pushes and pull requests, then
uploads the HTML as a downloadable ``bloom-documentation`` artifact. It does not
publish a public site or include participant recordings.
