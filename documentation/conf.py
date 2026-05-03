# SPDX-License-Identifier: Apache-2.0
"""Sphinx configuration for Torq — Breathe bridges Doxygen XML to Read the Docs."""

from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOXYFILE = ROOT / "Doxyfile"
DOXYGEN_XML = ROOT / "docs" / "xml"


def _run_doxygen() -> None:
    if not DOXYFILE.is_file():
        return
    subprocess.run(
        ["doxygen", str(DOXYFILE)],
        cwd=str(ROOT),
        check=True,
    )


_run_doxygen()

project = "Torq"
copyright = "Torq contributors"
author = "Torq contributors"
release = "0.1"

extensions = [
    "sphinx.ext.graphviz",  # required for class diagrams Breathe pulls from Doxygen XML
    "breathe",
    "sphinx_copybutton",
    "sphinx_wagtail_theme",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_wagtail_theme"
html_title = "Torq documentation"
html_show_sphinx = False

html_theme_options = {
    "project_name": "Torq",
    # Empty string overrides theme default; _templates/layout.html shows text-only brand.
    "logo": "",
    # Prefix for “View page source” (pagename + suffix are appended by the theme).
    "github_url": "https://github.com/Muhammad540/manipulation-stack/blob/main/documentation/",
    "footer_links": "",
    "header_links": "",
}

html_static_path = ["_static"]
html_css_files = ["custom.css"]

# Breathe: Doxygen XML under docs/xml (see Doxyfile OUTPUT_DIRECTORY + XML_OUTPUT).
breathe_projects = {"Torq": str(DOXYGEN_XML)}
breathe_default_project = "Torq"
