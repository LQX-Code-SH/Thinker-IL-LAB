"""Fix relative video paths on non-default locales.

Markdown images are rewritten by MkDocs, but raw HTML such as
``<video src="...">`` is passed through unchanged. Non-default locale pages
live one directory deeper (for example ``/en/walker-c1/``), so a relative
path authored for the default locale (``../assets/x.mp4``) would resolve to
``/en/assets/x.mp4`` instead of ``/assets/x.mp4``.

This hook adds one ``../`` to relative video/source paths on non-default
locales. Absolute URLs, root-relative paths, and fragments are left alone.
"""

from __future__ import annotations

import re

_VIDEO_SRC = re.compile(r'(<(?:video|source)\b[^>]*?\bsrc=")([^"]+)(")', re.IGNORECASE)
_ABSOLUTE_PREFIXES = ("http://", "https://", "//", "/", "#", "data:")


def on_page_content(html: str, page, config, files) -> str:
    locale = getattr(page.file, "locale", None)
    if not locale or locale == "zh":
        return html

    def repl(match: re.Match[str]) -> str:
        src = match.group(2)
        if src.startswith(_ABSOLUTE_PREFIXES):
            return match.group(0)
        return f"{match.group(1)}../{src}{match.group(3)}"

    return _VIDEO_SRC.sub(repl, html)
