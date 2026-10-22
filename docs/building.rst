Build the documentation
=======================

Run these commands from the Bloom repository root with Python 3.11 or newer::

   python3 -m venv docs/.venv
   docs/.venv/bin/python -m pip install -r docs/requirements.txt
   docs/.venv/bin/python -m sphinx -b html -c docs . docs/_build/html

Open ``docs/_build/html/docs/index.html`` to browse the generated documentation.
The RSR study guide is at ``docs/_build/html/docs/rsr-study.html``.
The HTML output, Python environment, and generated navigation catalog are ignored
by Git. Rebuild after editing documentation sources. The generated reference and release
indexes are also ignored by Git.

Sphinx's MyST parser turns Markdown guides into HTML pages. Study administration, development, and robot guides have curated navigation.
Remaining Markdown files are discovered automatically and grouped under Reference
and release notes. Sphinx
reStructuredText pages live in ``docs/``. Agent instructions, dependency trees,
private study archives, and generated files are excluded.

Publishing with GitHub Pages
----------------------------

The documentation workflow builds on pushes and pull requests and uploads a
``bloom-documentation`` artifact. It also publishes the selected branch to:

https://unr-roboticsresearchlab.github.io/bloom/

The site root opens the Sphinx documentation home page. Existing page paths,
including ``backend/BACKEND.html`` and ``docs/rsr-study.html``, remain available.
Only documentation and the static assets required by those pages are deployed;
participant recordings, local test credentials, and private study archives are
excluded from Sphinx sources.

In repository Settings, Pages must use **GitHub Actions** as its build source.
The repository Actions variable ``DOCS_PUBLISH_BRANCH`` selects the publishing
branch. It is initially set to ``rsr-study-dashboard`` for feature testing. After
merging, change it to ``main`` or delete it to use the repository default branch.
Pull requests build documentation but never deploy it. A push to any other branch
also builds without replacing the published site.

To trigger publication, push to the selected branch or use the Documentation
workflow's manual dispatch on that branch. GitHub's ``github-pages`` environment
must permit deployments from the selected branch. The deployment job grants
``pages: write`` and ``id-token: write`` only for publishing.
